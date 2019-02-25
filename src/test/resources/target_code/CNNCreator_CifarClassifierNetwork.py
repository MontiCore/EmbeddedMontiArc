import mxnet as mx
import logging
import os
import errno
import shutil
import h5py
import sys
import numpy as np
import time
from mxnet import gluon, autograd, nd
from CNNNet_CifarClassifierNetwork import Net

@mx.init.register
class MyConstant(mx.init.Initializer):
    def __init__(self, value):
        super(MyConstant, self).__init__(value=value)
        self.value = value
    def _init_weight(self, _, arr):
        arr[:] = mx.nd.array(self.value)

class CNNCreator_CifarClassifierNetwork:

    _data_dir_ = "data/CifarClassifierNetwork/"
    _model_dir_ = "model/CifarClassifierNetwork/"
    _model_prefix_ = "model"
    _input_names_ = ['data']
    _input_shapes_ = [(3,32,32)]
    _output_names_ = ['softmax_label']

    def __init__(self):
        self.weight_initializer = mx.init.Normal()
        self.net = None

    def load(self, context):
        lastEpoch = 0
        param_file = None

        try:
            os.remove(self._model_dir_ + self._model_prefix_ + "_newest-0000.params")
        except OSError:
            pass
        try:
            os.remove(self._model_dir_ + self._model_prefix_ + "_newest-symbol.json")
        except OSError:
            pass

        if os.path.isdir(self._model_dir_):
            for file in os.listdir(self._model_dir_):
                if ".params" in file and self._model_prefix_ in file:
                    epochStr = file.replace(".params","").replace(self._model_prefix_ + "-","")
                    epoch = int(epochStr)
                    if epoch > lastEpoch:
                        lastEpoch = epoch
                        param_file = file
        if param_file is None:
            return 0
        else:
            logging.info("Loading checkpoint: " + param_file)
            self.net.load_parameters(param_file)
            return lastEpoch


    def load_data(self, batch_size):
        train_h5, test_h5 = self.load_h5_files()

        data_mean = train_h5[self._input_names_[0]][:].mean(axis=0)
        data_std = train_h5[self._input_names_[0]][:].std(axis=0) + 1e-5

        train_iter = mx.io.NDArrayIter(train_h5[self._input_names_[0]],
                                       train_h5[self._output_names_[0]],
                                       batch_size=batch_size,
                                       data_name=self._input_names_[0],
                                       label_name=self._output_names_[0])
        test_iter = None
        if test_h5 != None:
            test_iter = mx.io.NDArrayIter(test_h5[self._input_names_[0]],
                                          test_h5[self._output_names_[0]],
                                          batch_size=batch_size,
                                          data_name=self._input_names_[0],
                                          label_name=self._output_names_[0])
        return train_iter, test_iter, data_mean, data_std

    def load_h5_files(self):
        train_h5 = None
        test_h5 = None
        train_path = self._data_dir_ + "train.h5"
        test_path = self._data_dir_ + "test.h5"
        if os.path.isfile(train_path):
            train_h5 = h5py.File(train_path, 'r')
            if not (self._input_names_[0] in train_h5 and self._output_names_[0] in train_h5):
                logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the datasets: "
                              + "'" + self._input_names_[0] + "', '" + self._output_names_[0] + "'")
                sys.exit(1)
            test_iter = None
            if os.path.isfile(test_path):
                test_h5 = h5py.File(test_path, 'r')
                if not (self._input_names_[0] in test_h5 and self._output_names_[0] in test_h5):
                    logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the datasets: "
                                  + "'" + self._input_names_[0] + "', '" + self._output_names_[0] + "'")
                    sys.exit(1)
            else:
                logging.warning("Couldn't load test set. File '" + os.path.abspath(test_path) + "' does not exist.")
            return train_h5, test_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(train_path) + "' does not exist.")
            sys.exit(1)


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


        train_iter, test_iter, data_mean, data_std = self.load_data(batch_size)
        if self.net == None:
            if normalize:
                self.construct(context=mx_context, data_mean=nd.array(data_mean), data_std=nd.array(data_std))
            else:
                self.construct(context=mx_context)

        begin_epoch = 0
        if load_checkpoint:
            begin_epoch = self.load(mx_context)
        else:
            if os.path.isdir(self._model_dir_):
                shutil.rmtree(self._model_dir_)

        try:
            os.makedirs(self._model_dir_)
        except OSError:
            if not os.path.isdir(self._model_dir_):
                raise

        trainer = mx.gluon.Trainer(self.net.collect_params(), optimizer, optimizer_params)

        if self.net.last_layer == 'softmax':
            loss_function = mx.gluon.loss.SoftmaxCrossEntropyLoss()
        elif self.net.last_layer == 'sigmoid':
            loss_function = mx.gluon.loss.SigmoidBinaryCrossEntropyLoss()
        elif self.net.last_layer == 'linear':
            loss_function = mx.gluon.loss.L2Loss()
        else: # TODO: Change default?
            loss_function = mx.gluon.loss.L2Loss()
            logging.warning("Invalid last_layer, defaulting to L2 loss")

        speed_period = 50
        tic = None

        for epoch in range(begin_epoch, begin_epoch + num_epoch):
            train_iter.reset()
            for batch_i, batch in enumerate(train_iter):
                data = batch.data[0].as_in_context(mx_context)
                label = batch.label[0].as_in_context(mx_context)
                with autograd.record():
                    output = self.net(data)
                    loss = loss_function(output, label)

                loss.backward()
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
                data = batch.data[0].as_in_context(mx_context)
                label = batch.label[0].as_in_context(mx_context)
                output = self.net(data)
                predictions = mx.nd.argmax(output, axis=1)
                metric.update(preds=predictions, labels=label)
            train_metric_score = metric.get()[1]

            test_iter.reset()
            metric = mx.metric.create(eval_metric)
            for batch_i, batch in enumerate(test_iter):
                data = batch.data[0].as_in_context(mx_context)
                label = batch.label[0].as_in_context(mx_context)
                output = self.net(data)
                predictions = mx.nd.argmax(output, axis=1)
                metric.update(preds=predictions, labels=label)
            test_metric_score = metric.get()[1]

            logging.info("Epoch[%d] Train: %f, Test: %f" % (epoch, train_metric_score, test_metric_score))

            if (epoch - begin_epoch) % checkpoint_period == 0:
                self.net.export(self._model_dir_ + self._model_prefix_, epoch)

        self.net.export(self._model_dir_ + self._model_prefix_, num_epoch + begin_epoch)
        self.net.export(self._model_dir_ + self._model_prefix_ + '_newest', 0)


    def construct(self, context, data_mean=None, data_std=None):
        self.net = Net(data_mean=data_mean, data_std=data_std)
        self.net.collect_params().initialize(self.weight_initializer, ctx=context)
        self.net.hybridize()
        self.net(mx.nd.zeros((1,)+self._input_shapes_[0], ctx=context))
