import mxnet as mx
import logging
import numpy as np
import time
import os
import shutil
from mxnet import gluon, autograd, nd

# ugly hardcoded
import  matplotlib as mpl
from matplotlib import pyplot as plt

def visualize(img_arr):
    plt.imshow((img_arr.asnumpy().transpose(1, 2, 0) * 255).astype(np.uint8).reshape(64,64))
    plt.axis('off')

def getDataIter(ctx, batch_size=64, Z=100):
    img_number = 70
    mnist_train = mx.gluon.data.vision.datasets.MNIST(train=True)
    mnist_test = mx.gluon.data.vision.datasets.MNIST(train=False)

    X = np.zeros((img_number, 28, 28))
    for i in range(img_number/2):
        X[i] = mnist_train[i][0].asnumpy()[:,:,0]
    for i in range(img_number/2):
        X[img_number/2+i] = mnist_test[i][0].asnumpy()[:,:,0]

    #X = np.zeros((img_number, 28, 28))
    #for i, (data, label) in enumerate(mnist_train):
    #    X[i] = mnist_train[i][0].asnumpy()[:,:,0]
    #for i, (data, label) in enumerate(mnist_test):
    #    X[len(mnist_train)+i] = data.asnumpy()[:,:,0]

    np.random.seed(1)
    p = np.random.permutation(X.shape[0])
    X = X[p]

    import cv2
    X = np.asarray([cv2.resize(x, (64,64)) for x in X])
    X = X.astype(np.float32, copy=False)/(255.0/2) - 1.0
    X = X.reshape((img_number, 1, 64, 64))
    X = np.tile(X, (1, 3, 1, 1))
    data = mx.nd.array(X)

    for i in range(4):
        plt.subplot(1,4,i+1)
        visualize(data[i])
    plt.show()

    image_iter = mx.io.NDArrayIter(data, batch_size=batch_size)
    return image_iter
# ugly hardcoded end

class CNNGanTrainer_rnnsearch_main_net:

    def __init__(self, data_loader, net_constructor_gen, net_constructor_dis):
        self._data_loader = data_loader
        self._net_creator_gen = net_constructor_gen
        self._net_creator_dis = net_constructor_dis

    def train(self, batch_size=64,
              num_epoch=10,
              eval_metric='acc',
              loss ='softmax_cross_entropy',
              loss_params={},
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=True,
              context='gpu',
              checkpoint_period=5,
              normalize=True,
              img_resize=(64,64),
              noise_distribution='gaussian',
              noise_distribution_params=(('mean_value', 0),('spread_value', 1),)):

        if context == 'gpu':
            mx_context = mx.gpu()
        elif context == 'cpu':
            mx_context = mx.cpu()
        else:
            logging.error("Context argument is '" + context + "'. Only 'cpu' and 'gpu are valid arguments'.")

        #train_iter = getDataIter(mx_context, batch_size, 100)
        train_iter, test_iter, data_mean, data_std = self._data_loader.load_data_img(batch_size, img_resize)

        if 'weight_decay' in optimizer_params:
            optimizer_params['wd'] = optimizer_params['weight_decay']
            del optimizer_params['weight_decay']
        if 'learning_rate_decay' in optimizer_params:
            min_learning_rate = 1e-08
            if 'learning_rate_minimum' in optimizer_params:
                min_learning_rate = optimizer_params['learning_rate_minimum']
                del optimizer_params['learning_rate_minimum']
            optimizer_params['lr_scheduler'] = mx.lr_scheduler.FactorScheduler(
                                                   optimizer_params['step_size'],
                                                  factor=optimizer_params['learning_rate_decay'],
                                                   stop_factor_lr=min_learning_rate)
            del optimizer_params['step_size']
            del optimizer_params['learning_rate_decay']

        if normalize:
            self._net_creator_dis.construct(mx_context, data_mean=data_mean, data_std=data_std)
        else:
            self._net_creator_dis.construct(mx_context)

        self._net_creator_gen.construct(mx_context)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self._net_creator_dis.load(mx_context)
            begin_epoch = self._net_creator_gen.load(mx_context)
        else:
            if os.path.isdir(self._net_creator_dis._model_dir_):
                shutil.rmtree(self._net_creator_dis._model_dir_)
            if os.path.isdir(self._net_creator_gen._model_dir_):
                shutil.rmtree(self._net_creator_gen._model_dir_)

        dis_net = self._net_creator_dis.networks[0]
        gen_net = self._net_creator_gen.networks[0]

        try:
            os.makedirs(self._net_creator_gen._model_dir_)
            os.makedirs(self._net_creator_dis._model_dir_)
        except OSError:
            if not (os.path.isdir(self._net_creator_gen._model_dir_) and
                    os.path.isdir(self._net_creator_dis._model_dir_)):
                raise

        gen_trainer = mx.gluon.Trainer(gen_net.collect_params(), optimizer, optimizer_params)
        dis_trainer = mx.gluon.Trainer(dis_net.collect_params(), optimizer, optimizer_params)

        if loss == 'sigmoid_binary_cross_entropy':
            loss_function = mx.gluon.loss.SigmoidBinaryCrossEntropyLoss()
            activation_name = 'sigmoid'

        input_shape = 302,1000,1
        input_shape = 1
        input_shape = 2,1000,1
        shape_list = list(input_shape)
        shape_list[0] = batch_size
        input_shape = tuple(shape_list)

        metric_dis = mx.metric.create(eval_metric)
        metric_gen = mx.metric.create(eval_metric)

        if noise_distribution == "gaussian":
            random_distributor = lambda : mx.ndarray.random.normal(noise_distribution_params["mean_value"],
                                                                    noise_distribution_params["spread_value"],
                                                                    shape=input_shape, ctx=mx_context)

        speed_period = 5
        tic = None

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            train_iter.reset()
            for batch_i, batch in enumerate(train_iter):
                real_data = batch.data[0].as_in_context(mx_context)
                rbatch = random_distributor()

                fake_labels = mx.nd.zeros((batch_size), ctx=mx_context)
                real_labels = mx.nd.ones((batch_size), ctx=mx_context)

                with autograd.record():
                    fake_data = gen_net(rbatch)
                    fake_data.detach()
                    discriminated_fake_dis = dis_net(fake_data)
                    loss_resultF = loss_function(discriminated_fake_dis, fake_labels)
                    discriminated_real_dis = dis_net(real_data)
                    loss_resultR = loss_function(discriminated_real_dis, real_labels)

                    loss_resultD = loss_resultR + loss_resultF
                loss_resultD.backward()
                dis_trainer.step(batch_size)


                with autograd.record():
                    fake_data = gen_net(rbatch)
                    discriminated_fake_gen = dis_net(fake_data)
                    loss_resultG = loss_function(discriminated_fake_gen, real_labels)
                loss_resultG.backward()
                gen_trainer.step(batch_size)

                if tic is None:
                    tic = time.time()
                else:
                    if batch_i % speed_period == 0:
                        metric_dis = mx.metric.create(eval_metric)
                        discriminated = mx.nd.Concat(discriminated_real_dis.reshape((-1,1)), discriminated_fake_dis.reshape((-1,1)), dim=0)
                        labels = mx.nd.Concat(real_labels.reshape((-1,1)), fake_labels.reshape((-1,1)), dim=0)
                        discriminated = mx.ndarray.Activation(discriminated, activation_name)
                        discriminated = mx.ndarray.floor(discriminated + 0.5)
                        metric_dis.update(preds=discriminated, labels=labels)
                        print("DisAcc: ", metric_dis.get()[1])

                        metric_gen = mx.metric.create(eval_metric)
                        discriminated = mx.ndarray.Activation(discriminated_fake_gen.reshape((-1,1)), activation_name)
                        discriminated = mx.ndarray.floor(discriminated + 0.5)
                        metric_gen.update(preds=discriminated, labels=real_labels.reshape((-1,1)))
                        print("GenAcc: ", metric_gen.get()[1])

                        try:
                            speed = speed_period * batch_size / (time.time() - tic)
                        except ZeroDivisionError:
                            speed = float("inf")

                        logging.info("Epoch[%d] Batch[%d] Speed: %.2f samples/sec" % (epoch, batch_i, speed))

                        tic = time.time()

                # ugly start
                #if batch_i % 20 == 0:
                #    fake_data[0].asnumpy()
                if batch_i % 50 == 0:
                    #gen_net.save_parameters(self.parameter_path_gen() + '-' + str(num_epoch + begin_epoch).zfill(4) + '.params')
                    #gen_net.export(self.parameter_path_gen() + '_newest', epoch=0)
                    #dis_net.save_parameters(self.parameter_path_dis() + '-' + str(num_epoch + begin_epoch).zfill(4) + '.params')
                    #dis_net.export(self.parameter_path_dis() + '_newest', epoch=0)
                    for i in range(10):
                        plt.subplot(1,10,i+1)
                        fake_img = fake_data[i]
                        visualize(fake_img)
                    plt.show()
                # ugly end


            if (epoch - begin_epoch) % checkpoint_period == 0:
                    gen_net.save_parameters(self.parameter_path_gen() + '-' + str(epoch).zfill(4) + '.params')
                    dis_net.save_parameters(self.parameter_path_dis() + '-' + str(epoch).zfill(4) + '.params')

        gen_net.save_parameters(self.parameter_path_gen() + '-' + str(num_epoch + begin_epoch).zfill(4) + '.params')
        gen_net.export(self.parameter_path_gen() + '_newest', epoch=0)
        dis_net.save_parameters(self.parameter_path_dis() + '-' + str(num_epoch + begin_epoch).zfill(4) + '.params')
        dis_net.export(self.parameter_path_dis() + '_newest', epoch=0)

    def parameter_path_gen(self):
        return self._net_creator_gen._model_dir_ + self._net_creator_gen._model_prefix_ + '_' + str(0)

    def parameter_path_dis(self):
        return self._net_creator_dis._model_dir_ + self._net_creator_dis._model_prefix_ + '_' + str(0)

