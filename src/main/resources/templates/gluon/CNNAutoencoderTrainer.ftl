import os
import shutil
import sys
import warnings

import mxnet as mx
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, autograd, gluon
from mxnet.gluon import nn

class GaussianVAELoss(gluon.loss.Loss):
    def __init__(self, weight=None, batch_axis=0, batch_size=1, model_ctx=mx.cpu(), **kwargs):
        super(VAELoss, self).__init__(weight,batch_axis,**kwargs)
        self.soft_zero = 1e-10
        self.batch_size = batch_size
        self.output = None
        self.mu = None
        self.model_ctx = model_ctx

    def hybrid_forward(self, F, pred, label, data, mean, std):
        x = data
        y = pred[0]
        mu = mean
        logvar = std

        KL = 0.5 * F.sum(1 + logvar - mu * mu - F.exp(logvar), axis=1)
        BCE = F.sum(x * F.log(y + self.soft_zero) + (1 - x) * F.log(1 - y + self.soft_zero), axis=1)
        loss = -BCE - KL
        return loss

class CNNAutoencoderTrainer_mnistvae_connector_dec:
    def __init__(self, data_loader, encoder_net_constructor, decoder_net_constructor):
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
              clip_global_grad_norm=None,
              print_images = False):
        num_pus = 1
        mx_context = [mx.cpu()]
        single_pu_batch_size = int(batch_size / num_pus)
        begin_epoch = 0
        train_iter, test_iter, train_data, test_data = self._data_loader.load_data(batch_size)

        self._enc_creator.construct(context=mx_context,train_data=train_data)
        self._dec_creator.construct(context=mx_context, train_data=train_data)
        encoder_nets = self._enc_creator.networks
        decoder_nets = self._dec_creator.networks

        enc_trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in
                    encoder_nets.values() if len(network.collect_params().values()) != 0]
        dec_trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in
                    decoder_nets.values() if len(network.collect_params().values()) != 0]

        loss_function = GaussianVAELoss(batch_size=batch_size)
        loss_function.hybridize()

        tic = None

        avg_speed = 0
        n = 0

        train_lost_list = []
        test_lost_list = []

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            global_loss_train = 0.0
            train_batches = 0

            loss_total = 0
            train_iter.reset()

            ls_output_list = []
            ls_label_list = []


            for batch_i, batch in enumerate(train_iter):

                with autograd.record():
                    labels = batch.label[0].as_in_context(mx_context[0])
                    data_ = batch.data[0].as_in_context(mx_context[0])

                    nd.waitall()
                    lossList = []
                    for i in range(num_pus):
                        lossList.append([])

                    nd.waitall()
                    latent_space_ = encoder_nets[0](data_)[0][0]
                    feature_mean, feature_logstd_sq = encoder_nets[0].latent_space_param_val
                    nd.waitall()
                    res_ = decoder_nets[0](latent_space_)

                    #tc.architecture.isLatentSpaceExisting() in template
                    [lossList[i].append(loss_function(res_[0],labels,data_,feature_mean,feature_logstd_sq)) for i in range(num_pus)]

                    #Otherwise
                    #[lossList[i].append(loss_function(res_[0], labels, data_)) for i in range(num_pus)]

                    losses = [0] * num_pus
                    for i in range(num_pus):
                        for element in lossList[i]:
                            losses[i] = losses[i] + element

                for loss in losses:
                    loss.backward()
                    loss_total += loss.sum().asscalar()
                    global_loss_train += loss.sum().asscalar()

                train_batches += 1

                for trainer in dec_trainers:
                    trainer.step(batch_size)
                for trainer in enc_trainers:
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

                ls_label_list.append(labels.copy())

                latent_space_ = encoder_nets[0](data_)[0][0]


                nd.waitall()
                latent_space_ = encoder_nets[0](data_)[0][0]
                feature_mean, feature_logstd_sq = encoder_nets[0].loss_param_val
                ls_output_list.append(feature_mean.copy())
                nd.waitall()
                res_ = decoder_nets[0](latent_space_)

                # tc.architecture.isLatentSpaceExisting() in template
                [lossList[i].append(loss_function(res_[0], labels, data_, feature_mean, feature_logstd_sq)) for i in range(num_pus)]

                # otherwise
                    #[lossList[i].append(loss_function(res_[0], labels, data_)) for i in range(num_pus)]

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

            if print_images:
                train_lost_list.append(global_loss_train)
                test_lost_list.append(global_loss_test)

        if print_images:


        if print_images:
            #Scatter Plot
            ls_outputs = mx.np.vstack(ls_output_list[0].as_np_ndarray())
            ls_labels = mx.np.hstack(ls_label_list[0].as_np_ndarray())

            filename = 'scatter_plot_%06d%06d.png' % (epoch, batch_i)
            fig, ax = plt.subplots()
            image = ax.scatter(ls_outputs[:, 0], ls_outputs[:, 1], c=ls_labels, alpha=0.5, cmap='Paired')
            ax.set_title(r'scatter plot of $\mu$')
            ax.axis('equal')
            fig.colorbar(image, ax=ax)
            plt.tight_layout()
            plt.savefig(filename)
            plt.show()
            plt.close()

            #Loss plot
            batch_x = np.linspace(1, num_epoch, len(train_lost_list))
            filename = 'loss_graph.png'
            plt.plot(batch_x, np.array(train_lost_list))
            plt.plot(batch_x, np.array(test_lost_list))
            plt.legend(['Train loss', 'Validation Loss'])
            plt.savefig(filename)


        for i, network in decoder_nets.items():
            network.save_parameters(self.decoder_parameter_path(i) + '-' + str((num_epoch-1) + begin_epoch).zfill(4) + '.params')
            network.export(self.decoder_parameter_path(i) + '_newest', epoch=0)

    def decoder_parameter_path(self, index):
        return self._dec_creator._model_dir_ + self._dec_creator._model_prefix_ + '_' + str(index)