import mxnet as mx
import logging
import numpy as np
import time
import os
import shutil
from mxnet import gluon, autograd, nd

class CNNSupervisedTrainer_CifarClassifierNetwork:
    def __init__(self, data_loader, net_constructor):
        self._data_loader = data_loader
        self._net_creator = net_constructor
        self._networks = {}

    def train(self, batch_size=64,
              num_epoch=10,
              eval_metric='acc',
              optimizer='adam',
              optimizer_params=(('learning_rate', 0.001),),
              load_checkpoint=True,
              context='gpu',
              checkpoint_period=5,
              normalize=True):
        if context == 'gpu':
            mx_context = mx.gpu()
        elif context == 'cpu':
            mx_context = mx.cpu()
        else:
            logging.error("Context argument is '" + context + "'. Only 'cpu' and 'gpu are valid arguments'.")

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


        train_iter, test_iter, data_mean, data_std = self._data_loader.load_data(batch_size)

        if normalize:
            self._net_creator.construct(context=mx_context, data_mean=data_mean, data_std=data_std)
        else:
            self._net_creator.construct(context=mx_context)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self._net_creator.load(mx_context)
        else:
            if os.path.isdir(self._net_creator._model_dir_):
                shutil.rmtree(self._net_creator._model_dir_)

        self._networks = self._net_creator.networks

        try:
            os.makedirs(self._net_creator._model_dir_)
        except OSError:
            if not os.path.isdir(self._net_creator._model_dir_):
                raise

        trainers = [mx.gluon.Trainer(network.collect_params(), optimizer, optimizer_params) for network in self._networks.values()]

        loss_functions = {}

        for network in self._networks.values():
            for output_name, last_layer in network.last_layers.items():
                if last_layer == 'softmax':
                    loss_functions[output_name] = mx.gluon.loss.SoftmaxCrossEntropyLoss()
                elif last_layer == 'sigmoid':
                    loss_functions[output_name] = mx.gluon.loss.SigmoidBinaryCrossEntropyLoss()
                elif last_layer == 'linear':
                    loss_functions[output_name] = mx.gluon.loss.L2Loss()
                else:
                    loss_functions[output_name] = mx.gluon.loss.L2Loss()
                    logging.warning("Invalid last layer, defaulting to L2 loss")

        speed_period = 50
        tic = None

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            train_iter.reset()
            for batch_i, batch in enumerate(train_iter):
                data_data = batch.data[0].as_in_context(mx_context)
                softmax_label = batch.label[0].as_in_context(mx_context)

                with autograd.record():
                    softmax_output = self._networks[0](data_data)

                    loss = loss_functions['softmax'](softmax_output, softmax_label)

                loss.backward()

                for trainer in trainers:
                    trainer.step(batch_size)

                if tic is None:
                    tic = time.time()
                else:
                    if batch_i % speed_period == 0:
                        try:
                            speed = speed_period * batch_size / (time.time() - tic)
                        except ZeroDivisionError:
                            speed = float("inf")

                        logging.info("Epoch[%d] Batch[%d] Speed: %.2f samples/sec" % (epoch, batch_i, speed))

                        tic = time.time()

            tic = None

            train_iter.reset()
            metric = mx.metric.create(eval_metric)
            for batch_i, batch in enumerate(train_iter):
                data_data = batch.data[0].as_in_context(mx_context)

                labels = [
                    batch.label[0].as_in_context(mx_context)
                ]

                softmax_output = self._networks[0](data_data)

                predictions = [
                    mx.nd.argmax(softmax_output, axis=1)
                ]

                metric.update(preds=predictions, labels=labels)
            train_metric_score = metric.get()[1]

            test_iter.reset()
            metric = mx.metric.create(eval_metric)
            for batch_i, batch in enumerate(test_iter):
                data_data = batch.data[0].as_in_context(mx_context)

                labels = [
                    batch.label[0].as_in_context(mx_context)
                ]

                softmax_output = self._networks[0](data_data)
                predictions = [
                    mx.nd.argmax(softmax_output, axis=1)
                ]

                metric.update(preds=predictions, labels=labels)
            test_metric_score = metric.get()[1]

            logging.info("Epoch[%d] Train: %f, Test: %f" % (epoch, train_metric_score, test_metric_score))

            if (epoch - begin_epoch) % checkpoint_period == 0:
                for i, network in self._networks.items():
                    network.save_parameters(self.parameter_path(i) + '-' + str(epoch).zfill(4) + '.params')

        for i, network in self._networks.items():
            network.save_parameters(self.parameter_path(i) + '-' + str(num_epoch + begin_epoch).zfill(4) + '.params')
            network.export(self.parameter_path(i) + '_newest', epoch=0)

    def parameter_path(self, index):
        return self._net_creator._model_dir_ + self._net_creator._model_prefix_ + '_' + str(index)