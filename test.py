import os
import shutil
import warnings

import mxnet as mx
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, autograd, gluon
from mxnet.gluon import nn


class CNNCreator_mnistvae_connector_enc:
    _model_dir_ = "model/mnistvae.Network/"
    _model_prefix_ = "model"

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
        self.networks = {}
        self._weights_dir_ = None

    def construct(self, context, data_mean=None, data_std=None, train_data=None):
        self.networks[0] = Net_0(data_mean=data_mean, data_std=data_std, mx_context=context, prefix="")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.networks[0].collect_params().initialize(self.weight_initializer, force_reinit=False, ctx=context)
            self.networks[0].summary(mx.nd.array(train_data[0]))
        self.networks[0].hybridize()
        self.networks[0](mx.nd.zeros((1, 1,28,28), ctx=context[0]))

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

        for i, network in self.networks.items():
            network.export(self._model_dir_ + self._model_prefix_ + "_" + str(i), epoch=0)


class CNNCreator_mnistvae_connector_dec:
    _model_dir_ = "model/mnistvae.Network/"
    _model_prefix_ = "model"

    def __init__(self,encoder):
        self.weight_initializer = mx.init.Normal()
        self.networks = {}
        self._weights_dir_ = None
        self._encoder = encoder.networks
        self.latent_space = None

    def construct(self, context, data_mean=None, data_std=None, train_data=None):
        self.networks[0] = Net_0(data_mean=data_mean, data_std=data_std, mx_context=context, prefix="")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.networks[0].collect_params().initialize(self.weight_initializer, force_reinit=False, ctx=context)
            self.networks[0].summary(mx.nd.array(train_data[0]))
        self.networks[0].hybridize()
        decoder_input, self.latent_space = self._encoder[0](mx.nd.zeros((1, 1, 28, 28), ctx=context[0]))
        self.networks[0](decoder_input[0][0])

        if not os.path.exists(self._model_dir_):
            os.makedirs(self._model_dir_)

        for i, network in self.networks.items():
            network.export(self._model_dir_ + self._model_prefix_ + "_" + str(i), epoch=0)

class CNNDataLoader_mnistvae_connector_enc:
    def __init__(self):
        self._data_dir = "resources/resources/training_data/"

    def load_data(self, batch_size):
        mnist = mx.test_utils.get_mnist()

        train_data = np.reshape(mnist['train_data'], (-1, 784))
        test_data = np.reshape(mnist['test_data'], (-1, 784))

        train_iter = mx.io.NDArrayIter(data={'data': train_data}, label={'label': mnist['train_label']},
                                       batch_size=batch_size)
        test_iter = mx.io.NDArrayIter(data={'data': test_data}, label={'label': mnist['test_label']},
                                      batch_size=batch_size)

        return train_iter, test_iter, train_data, test_data

class NoNormalization(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(NoNormalization, self).__init__(**kwargs)

    def hybrid_forward(self, F, x):
        return x

class Net_0(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, mx_context=None, **kwargs):
        super(Net_0, self).__init__(**kwargs)
        with self.name_scope():
            self.batch_size = None
            self.input_normalization_data_ = NoNormalization()
            self.fc1_ = gluon.nn.Dense(400,activation='relu')
            self.fc2_ = gluon.nn.Dense(4,activation=None)
            self.fc3_ = gluon.nn.Dense(400,activation='relu')
            self.fc4_ = gluon.nn.Dense(784,activation='sigmoid')

            pass

    def hybrid_forward(self, F, data_):
        data_ = self.input_normalization_data_(data_)
        reshape = F.reshape(data_,shape=(32,784))
        fc1_ = self.fc1_(reshape)
        fc2_ = self.fc2_(fc1_)

        split2_ = F.split(fc2_, axis=1, num_outputs=2)
        get3_1_ = split2_[0]
        z_mu_ = F.identity(get3_1_)
        get3_2_ = split2_[1]
        z_logvar_ = F.identity(get3_2_)

        eps = F.random_normal(loc=0, scale=1, shape=(32,2) , ctx=mx.cpu())
        reparametrization_ = z_mu_ + F.exp(0.5 * z_logvar_) * eps

        fc3_ = self.fc3_(reparametrization_)
        fc4_ = self.fc4_(fc3_)

        res_ = F.identity(fc4_)

        return [[res_]], [z_mu_,z_logvar_]

class VAELoss(gluon.loss.Loss):
    def __init__(self, weight=None, batch_axis=0, batch_size=32, model_ctx=mx.cpu(), **kwargs):
        super(VAELoss, self).__init__(weight,batch_axis,**kwargs)
        self.soft_zero = 1e-10
        self.batch_size = batch_size
        self.output = None
        self.mu = None
        self.model_ctx = model_ctx

    def hybrid_forward(self, F, pred, label, data, latent_space):
        x = data
        y = pred[0][0]
        mu = latent_space[0]
        logvar = latent_space[1]

        KL = 0.5 * F.sum(1 + logvar - mu * mu - F.exp(logvar), axis=1)
        BCE = F.sum(x * F.log(y + self.soft_zero) + (1 - x) * F.log(1 - y + self.soft_zero), axis=1)
        loss = -BCE - KL
        return loss

class CNNAutoencoderTrainer_mnistvae_connector_dec:
    def __init__(self, data_loader, encoder_net_constructor, decoder_net_constructor,):
        self._data_loader = data_loader
        self._enc_creator = encoder_net_constructor
        self._dec_creator = decoder_net_constructor

    def train(self, batch_size=64,
              num_epoch=10,
              eval_metric='acc',
              eval_metric_params={},
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              log_period=50,
              clip_global_grad_norm=None):
        num_pus = 1
        mx_context = [mx.cpu()]
        single_pu_batch_size = int(batch_size / num_pus)
        begin_epoch = 0
        train_iter, test_iter, train_data, test_data = self._data_loader.load_data(batch_size)

        self._enc_creator.construct(context=mx_context,train_data=train_data)
        self._dec_creator.construct(context=mx_context, train_data=train_data)
        encoder_net = self._enc_creator.networks[0]
        decoder_net = self._dec_creator.networks


        trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in
                    decoder_net.values() if len(network.collect_params().values()) != 0]

        loss_function = VAELoss(batch_size=batch_size)
        loss_function.hybridize()

        tic = None

        avg_speed = 0
        n = 0

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            global_loss_train = 0.0
            train_batches = 0

            loss_total = 0
            train_iter.reset()
            for batch_i, batch in enumerate(train_iter):

                with autograd.record():
                    labels = batch.label[0].as_in_context(mx_context[0])
                    data_ = batch.data[0].as_in_context(mx_context[0])

                    nd.waitall()
                    lossList = []
                    for i in range(num_pus):
                        lossList.append([])

                    res_,_ = decoder_net[0](data_)
                    latent_space_ = self._dec_creator.latent_space
                    if latent_space_:
                        [lossList[i].append(loss_function(res_[0],labels,data_,latent_space_)) for i in range(num_pus)]
                    else:
                        [lossList[i].append(loss_function(res_[0], labels, data_)) for i in range(num_pus)]

                    losses = [0] * num_pus
                    for i in range(num_pus):
                        for element in lossList[i]:
                            losses[i] = losses[i] + element

                for loss in losses:
                    loss.backward()
                    loss_total += loss.sum().asscalar()
                    global_loss_train += loss.sum().asscalar()

                train_batches += 1

                for trainer in trainers:
                    trainer.step(batch_size)

                if tic is None:
                    tic = time.time()
                else:
                    if batch_i % log_period == 0:
                        try:
                            speed = log_period * batch_size / (time.time() - tic)
                        except ZeroDivisionError:
                            speed = float("inf")

                        loss_avg = loss_total / (batch_size * log_period)
                        loss_total = 0

                        logging.info("Epoch[%d] Batch[%d] Speed: %.2f samples/sec Average Loss: %.5f" % (
                        epoch, batch_i, speed, loss_avg))

                        avg_speed += speed
                        n += 1

                        tic = time.time()

            global_loss_train /= (train_batches * batch_size)

            tic = None

            global_loss_test = 0.0
            test_batches = 0

            test_iter.batch_size = single_pu_batch_size
            test_iter.reset()

            for batch_i, batch in enumerate(test_iter):
                labels = batch.label[0].as_in_context(mx_context[0])
                data_ = batch.data[0].as_in_context(mx_context[0])

                nd.waitall()
                lossList = []
                for i in range(num_pus):
                    lossList.append([])

                res_, _ = decoder_net[0](data_)
                latent_space_ = self._dec_creator.latent_space
                if latent_space_:
                    [lossList[i].append(loss_function(res_[0], labels, data_, latent_space_)) for i in range(num_pus)]
                else:
                    [lossList[i].append(loss_function(res_[0], labels, data_)) for i in range(num_pus)]

                losses = [0] * num_pus
                for i in range(num_pus):
                    for element in lossList[i]:
                        losses[i] = losses[i] + element

                for loss in losses:
                    global_loss_test += loss.sum().asscalar()

                global_loss_test += loss.sum().asscalar()

                test_batches += 1

            global_loss_test /= (test_batches * single_pu_batch_size)

            logging.info("Epoch[%d], Train loss: %f, Validation loss: %f" % (
                          epoch, global_loss_train, global_loss_test))


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    mnistvae_connector_enc_creator = CNNCreator_mnistvae_connector_enc()
    mnistvae_connector_dec_creator = CNNCreator_mnistvae_connector_dec(mnistvae_connector_enc_creator)
    mnistvae_connector_enc_loader = CNNDataLoader_mnistvae_connector_enc()
    mnistvae_connector_dec_trainer = CNNAutoencoderTrainer_mnistvae_connector_dec(
        mnistvae_connector_enc_loader,
        mnistvae_connector_enc_creator,
        mnistvae_connector_dec_creator
    )

    mnistvae_connector_dec_trainer.train(
        batch_size=32,
        num_epoch=3,
        optimizer='adam',
        optimizer_params={
            'learning_rate': 3.0E-5}
    )