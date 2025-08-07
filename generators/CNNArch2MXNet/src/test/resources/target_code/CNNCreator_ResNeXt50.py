# (c) https://github.com/MontiCore/monticore
import mxnet as mx
import logging
import os, gc
import errno
import shutil
import h5py
import sys
import numpy as np

@mx.init.register
class MyConstant(mx.init.Initializer):
    def __init__(self, value):
        super(MyConstant, self).__init__(value=value)
        self.value = value
    def _init_weight(self, _, arr):
        arr[:] = mx.nd.array(self.value)

class CNNCreator_ResNeXt50:

    def __init__(self, data_cleaner):
        self.module = None
        self._data_dir_ = "data/ResNeXt50/"
        self._model_dir_ = "model/ResNeXt50/"
        self._model_prefix_ = "model"
        self._data_cleaner_ = data_cleaner

        self._input_shapes_ = [(3,224,224)]
        self._input_names_ = ['data_']
        self._output_names_ = ['predictions__label']
        self._input_data_names_ = ['data']
        self._output_data_names_ = ['predictions_label']


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
            self.module.load(prefix=self._model_dir_ + self._model_prefix_,
                              epoch=lastEpoch,
                              data_names=self._input_names_,
                              label_names=self._output_names_,
                              context=context)
            return lastEpoch


    def load_data(self, batch_size, cleaning, cleaning_params, data_imbalance, data_imbalance_params):
        train_h5, test_h5 = self.load_h5_files()

        if cleaning == 'remove':
            train_data, train_label, test_data, test_label = self._data_cleaner_.clean_data( train_h5, test_h5, cleaning, cleaning_params )
            
            if data_imbalance == 'image_augmentation':
                train_data, train_label = self._data_cleaner_.image_augmentation( train_data, train_label, data_imbalance_params )
                test_data, test_label = self._data_cleaner_.image_augmentation( test_data, test_label, data_imbalance_params )
        
            train_h5, test_h5 = self._data_cleaner_.numpy_to_hdf5(train_data, train_label, test_data, test_label)
            os.remove('_train.h5')
            os.remove('_test.h5')

        data_mean = train_h5[self._input_data_names_[0]][:].mean(axis=0)
        data_std = train_h5[self._input_data_names_[0]][:].std(axis=0) + 1e-5

        train_iter = mx.io.NDArrayIter(train_h5[self._input_data_names_[0]],
                                       train_h5[self._output_data_names_[0]],
                                       batch_size=batch_size,
                                       data_name=self._input_names_[0],
                                       label_name=self._output_names_[0], shuffle=True)
        test_iter = None
        if test_h5 != None:
            test_iter = mx.io.NDArrayIter(test_h5[self._input_data_names_[0]],
                                          test_h5[self._output_data_names_[0]],
                                          batch_size=batch_size,
                                          data_name=self._input_names_[0],
                                          label_name=self._output_names_[0], shuffle=True)
        return train_iter, test_iter, data_mean, data_std

    def load_h5_files(self):
        train_h5 = None
        test_h5 = None
        train_path = self._data_dir_ + "train.h5"
        test_path = self._data_dir_ + "test.h5"
        if os.path.isfile(train_path):
            train_h5 = h5py.File(train_path, 'r')
            if not (self._input_data_names_[0] in train_h5 and self._output_data_names_[0] in train_h5):
                logging.error("The HDF5 file '" + os.path.abspath(train_path) + "' has to contain the datasets: "
                              + "'" + self._input_data_names_[0] + "', '" + self._output_data_names_[0] + "'")
                sys.exit(1)
            test_iter = None
            if os.path.isfile(test_path):
                test_h5 = h5py.File(test_path, 'r')
                if not (self._input_data_names_[0] in test_h5 and self._output_data_names_[0] in test_h5):
                    logging.error("The HDF5 file '" + os.path.abspath(test_path) + "' has to contain the datasets: "
                                  + "'" + self._input_data_names_[0] + "', '" + self._output_data_names_[0] + "'")
                    sys.exit(1)
            else:
                logging.warning("Couldn't load test set. File '" + os.path.abspath(test_path) + "' does not exist.")
            return train_h5, test_h5
        else:
            logging.error("Data loading failure. File '" + os.path.abspath(train_path) + "' does not exist.")
            sys.exit(1)

    def loss_function(self, loss, params):
        label = mx.symbol.var(name=self._output_names_[0], )
        prediction = self.module.symbol.get_children()[0]

        margin = params['margin'] if 'margin' in params else 1.0
        sparseLabel = params['sparse_label'] if 'sparse_label' in params else True

        if loss == 'softmax_cross_entropy':
            fromLogits = params['from_logits'] if 'from_logits' in params else False
            if not fromLogits:
                loss_funct = mx.symbol.log_softmax(data=prediction, axis=1)
            if sparseLabel:
                loss_func = mx.symbol.mean(-mx.symbol.pick(prediction, label, axis=-1, keepdims=True), axis=0, exclude=True)
            else:
                label = mx.symbol.reshape_like(label, prediction)
                loss_func = mx.symbol.mean(-mx.symbol.sum(prediction * label, axis=-1, keepdims=True), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="softmax_cross_entropy")
        elif loss == 'cross_entropy':
            prediction = mx.symbol.log(prediction)
            if sparseLabel:
                loss_func = mx.symbol.mean(-mx.symbol.pick(prediction, label, axis=-1, keepdims=True), axis=0, exclude=True)
            else:
                label = mx.symbol.reshape_like(label, prediction)
                loss_func = mx.symbol.mean(-mx.symbol.sum(prediction * label, axis=-1, keepdims=True), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="cross_entropy")
        elif loss == 'sigmoid_binary_cross_entropy':
            loss_func = mx.symbol.LogisticRegressionOutput(data=prediction, name=self.module.symbol.name)
        elif loss == 'l1':
            loss_func = mx.symbol.MAERegressionOutput(data=prediction, name=self.module.symbol.name)
        elif loss == 'l2':
            label = mx.symbol.reshape_like(label, prediction)
            loss_func = mx.symbol.mean(mx.symbol.square((label - prediction) / 2), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="L2")
        elif loss == 'huber':
            rho = params['rho'] if 'rho' in params else 1
            label = mx.symbol.reshape_like(label, prediction)
            loss_func = mx.symbol.abs(label - prediction)
            loss_func = mx.symbol.where(loss_func > rho, loss_func - 0.5 * rho, (0.5 / rho) * mx.symbol.square(loss_func))
            loss_func = mx.symbol.mean(loss_func, axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="huber")
        elif loss == 'hinge':
            label = mx.symbol.reshape_like(label, prediction)
            loss_func = mx.symbol.mean(mx.symbol.relu(margin - prediction * label), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="hinge")
        elif loss == 'squared_hinge':
            label = mx.symbol.reshape_like(label, prediction)
            loss_func = mx.symbol.mean(mx.symbol.square(mx.symbol.relu(margin - prediction * label)), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="squared_hinge")
        elif loss == 'logistic':
            labelFormat = params['label_format'] if 'label_format' in params else 'signed'
            if labelFormat not in ["binary", "signed"]:
                logging.error("label_format can only be signed or binary")
            label = mx.symbol.reshape_like(label, prediction)
            if labelFormat == 'signed':
                label = (label + 1.0)/2.0
            loss_func = mx.symbol.relu(prediction) - prediction * label
            loss_func = loss_func + mx.symbol.Activation(-mx.symbol.abs(prediction), act_type="softrelu")
            loss_func = mx.symbol.MakeLoss(mx.symbol.mean(loss_func, 0, exclude=True), name="logistic")
        elif loss == 'kullback_leibler':
            fromLogits = params['from_logits'] if 'from_logits' in params else True
            if not fromLogits:
                prediction = mx.symbol.log_softmax(prediction, axis=1)
            loss_func = mx.symbol.mean(label * (mx.symbol.log(label) - prediction), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="kullback_leibler")
        elif loss == 'log_cosh':
            loss_func = mx.symbol.mean(mx.symbol.log(mx.symbol.cosh(prediction - label)), axis=0, exclude=True)
            loss_func = mx.symbol.MakeLoss(loss_func, name="log_cosh")
        else:
            logging.error("Invalid loss parameter.")

        return loss_func

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
              cleaning=None,
              cleaning_params=(None),
              data_imbalance=None,
              data_imbalance_params=(None)):
              
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

        train_iter, test_iter, data_mean, data_std = self.load_data(batch_size, cleaning, cleaning_params, data_imbalance, data_imbalance_params)
        
        if self.module == None:
            if normalize:
                self.construct(mx_context, data_mean, data_std)
            else:
                self.construct(mx_context)

        loss_func = self.loss_function(loss=loss, params=loss_params)

        self.module = mx.mod.Module(
            symbol=mx.symbol.Group([loss_func, mx.symbol.BlockGrad(self.module.symbol.get_children()[0], name="pred")]),
            data_names=self._input_names_,
            label_names=self._output_names_,
            context=mx_context)

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

        metric = mx.metric.create(eval_metric, output_names=['pred_output'])

        self.module.fit(
            train_data=train_iter,
            eval_metric=metric,
            eval_data=test_iter,
            optimizer=optimizer,
            optimizer_params=optimizer_params,
            batch_end_callback=mx.callback.Speedometer(batch_size),
            epoch_end_callback=mx.callback.do_checkpoint(prefix=self._model_dir_ + self._model_prefix_, period=checkpoint_period),
            begin_epoch=begin_epoch,
            num_epoch=num_epoch + begin_epoch)
        
        self.module.save_checkpoint(self._model_dir_ + self._model_prefix_, num_epoch + begin_epoch)
        self.module.save_checkpoint(self._model_dir_ + self._model_prefix_ + '_newest', 0)

        if data_imbalance is not None:
            if data_imbalance_params['check_bias']:
                _, test = self.load_h5_files()
                self._data_cleaner_.check_bias(np.array(test["data"]), np.array(test["softmax_label"]).astype(int), self._model_dir_)
                self._data_cleaner_.check_bias(np.array(test["data"]), np.array(test["softmax_label"]).astype(int), self._model_dir_)


    def construct(self, context, data_mean=None, data_std=None):
        data_ = mx.sym.var("data_",
            shape=(1,3,224,224))
        # data_, output shape: {[3,224,224]}

        if not data_mean is None:
            assert(not data_std is None)
            _data_mean_ = mx.sym.Variable("_data_mean_", shape=(3,224,224), init=MyConstant(value=data_mean.tolist()))
            _data_mean_ = mx.sym.BlockGrad(_data_mean_)
            _data_std_ = mx.sym.Variable("_data_std_", shape=(3,224,224), init=MyConstant(value=data_mean.tolist()))
            _data_std_ = mx.sym.BlockGrad(_data_std_)
            data_ = mx.symbol.broadcast_sub(data_, _data_mean_)
            data_ = mx.symbol.broadcast_div(data_, _data_std_)
        conv1_ = mx.symbol.pad(data=data_,
            mode='constant',
            pad_width=(0,0,0,0,3,2,3,2),
            constant_value=0)
        conv1_ = mx.symbol.Convolution(data=conv1_,
            kernel=(7,7),
            stride=(2,2),
            num_filter=64,
            no_bias=False,
            name="conv1_")
        # conv1_, output shape: {[64,112,112]}

        batchnorm1_ = mx.symbol.BatchNorm(data=conv1_,
            fix_gamma=True,
            name="batchnorm1_")
        relu1_ = mx.symbol.Activation(data=batchnorm1_,
            act_type='relu',
            name="relu1_")

        pool1_ = mx.symbol.pad(data=relu1_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        pool1_ = mx.symbol.Pooling(data=pool1_,
            kernel=(3,3),
            pool_type="max",
            stride=(2,2),
            name="pool1_")
        # pool1_, output shape: {[64,56,56]}

        conv3_1_1_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_1_")
        # conv3_1_1_, output shape: {[4,56,56]}

        batchnorm3_1_1_ = mx.symbol.BatchNorm(data=conv3_1_1_,
            fix_gamma=True,
            name="batchnorm3_1_1_")
        relu3_1_1_ = mx.symbol.Activation(data=batchnorm3_1_1_,
            act_type='relu',
            name="relu3_1_1_")

        conv4_1_1_ = mx.symbol.pad(data=relu3_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_1_ = mx.symbol.Convolution(data=conv4_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_1_")
        # conv4_1_1_, output shape: {[4,56,56]}

        batchnorm4_1_1_ = mx.symbol.BatchNorm(data=conv4_1_1_,
            fix_gamma=True,
            name="batchnorm4_1_1_")
        relu4_1_1_ = mx.symbol.Activation(data=batchnorm4_1_1_,
            act_type='relu',
            name="relu4_1_1_")

        conv5_1_1_ = mx.symbol.Convolution(data=relu4_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_1_")
        # conv5_1_1_, output shape: {[256,56,56]}

        batchnorm5_1_1_ = mx.symbol.BatchNorm(data=conv5_1_1_,
            fix_gamma=True,
            name="batchnorm5_1_1_")
        conv3_1_2_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_2_")
        # conv3_1_2_, output shape: {[4,56,56]}

        batchnorm3_1_2_ = mx.symbol.BatchNorm(data=conv3_1_2_,
            fix_gamma=True,
            name="batchnorm3_1_2_")
        relu3_1_2_ = mx.symbol.Activation(data=batchnorm3_1_2_,
            act_type='relu',
            name="relu3_1_2_")

        conv4_1_2_ = mx.symbol.pad(data=relu3_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_2_ = mx.symbol.Convolution(data=conv4_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_2_")
        # conv4_1_2_, output shape: {[4,56,56]}

        batchnorm4_1_2_ = mx.symbol.BatchNorm(data=conv4_1_2_,
            fix_gamma=True,
            name="batchnorm4_1_2_")
        relu4_1_2_ = mx.symbol.Activation(data=batchnorm4_1_2_,
            act_type='relu',
            name="relu4_1_2_")

        conv5_1_2_ = mx.symbol.Convolution(data=relu4_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_2_")
        # conv5_1_2_, output shape: {[256,56,56]}

        batchnorm5_1_2_ = mx.symbol.BatchNorm(data=conv5_1_2_,
            fix_gamma=True,
            name="batchnorm5_1_2_")
        conv3_1_3_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_3_")
        # conv3_1_3_, output shape: {[4,56,56]}

        batchnorm3_1_3_ = mx.symbol.BatchNorm(data=conv3_1_3_,
            fix_gamma=True,
            name="batchnorm3_1_3_")
        relu3_1_3_ = mx.symbol.Activation(data=batchnorm3_1_3_,
            act_type='relu',
            name="relu3_1_3_")

        conv4_1_3_ = mx.symbol.pad(data=relu3_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_3_ = mx.symbol.Convolution(data=conv4_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_3_")
        # conv4_1_3_, output shape: {[4,56,56]}

        batchnorm4_1_3_ = mx.symbol.BatchNorm(data=conv4_1_3_,
            fix_gamma=True,
            name="batchnorm4_1_3_")
        relu4_1_3_ = mx.symbol.Activation(data=batchnorm4_1_3_,
            act_type='relu',
            name="relu4_1_3_")

        conv5_1_3_ = mx.symbol.Convolution(data=relu4_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_3_")
        # conv5_1_3_, output shape: {[256,56,56]}

        batchnorm5_1_3_ = mx.symbol.BatchNorm(data=conv5_1_3_,
            fix_gamma=True,
            name="batchnorm5_1_3_")
        conv3_1_4_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_4_")
        # conv3_1_4_, output shape: {[4,56,56]}

        batchnorm3_1_4_ = mx.symbol.BatchNorm(data=conv3_1_4_,
            fix_gamma=True,
            name="batchnorm3_1_4_")
        relu3_1_4_ = mx.symbol.Activation(data=batchnorm3_1_4_,
            act_type='relu',
            name="relu3_1_4_")

        conv4_1_4_ = mx.symbol.pad(data=relu3_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_4_ = mx.symbol.Convolution(data=conv4_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_4_")
        # conv4_1_4_, output shape: {[4,56,56]}

        batchnorm4_1_4_ = mx.symbol.BatchNorm(data=conv4_1_4_,
            fix_gamma=True,
            name="batchnorm4_1_4_")
        relu4_1_4_ = mx.symbol.Activation(data=batchnorm4_1_4_,
            act_type='relu',
            name="relu4_1_4_")

        conv5_1_4_ = mx.symbol.Convolution(data=relu4_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_4_")
        # conv5_1_4_, output shape: {[256,56,56]}

        batchnorm5_1_4_ = mx.symbol.BatchNorm(data=conv5_1_4_,
            fix_gamma=True,
            name="batchnorm5_1_4_")
        conv3_1_5_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_5_")
        # conv3_1_5_, output shape: {[4,56,56]}

        batchnorm3_1_5_ = mx.symbol.BatchNorm(data=conv3_1_5_,
            fix_gamma=True,
            name="batchnorm3_1_5_")
        relu3_1_5_ = mx.symbol.Activation(data=batchnorm3_1_5_,
            act_type='relu',
            name="relu3_1_5_")

        conv4_1_5_ = mx.symbol.pad(data=relu3_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_5_ = mx.symbol.Convolution(data=conv4_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_5_")
        # conv4_1_5_, output shape: {[4,56,56]}

        batchnorm4_1_5_ = mx.symbol.BatchNorm(data=conv4_1_5_,
            fix_gamma=True,
            name="batchnorm4_1_5_")
        relu4_1_5_ = mx.symbol.Activation(data=batchnorm4_1_5_,
            act_type='relu',
            name="relu4_1_5_")

        conv5_1_5_ = mx.symbol.Convolution(data=relu4_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_5_")
        # conv5_1_5_, output shape: {[256,56,56]}

        batchnorm5_1_5_ = mx.symbol.BatchNorm(data=conv5_1_5_,
            fix_gamma=True,
            name="batchnorm5_1_5_")
        conv3_1_6_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_6_")
        # conv3_1_6_, output shape: {[4,56,56]}

        batchnorm3_1_6_ = mx.symbol.BatchNorm(data=conv3_1_6_,
            fix_gamma=True,
            name="batchnorm3_1_6_")
        relu3_1_6_ = mx.symbol.Activation(data=batchnorm3_1_6_,
            act_type='relu',
            name="relu3_1_6_")

        conv4_1_6_ = mx.symbol.pad(data=relu3_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_6_ = mx.symbol.Convolution(data=conv4_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_6_")
        # conv4_1_6_, output shape: {[4,56,56]}

        batchnorm4_1_6_ = mx.symbol.BatchNorm(data=conv4_1_6_,
            fix_gamma=True,
            name="batchnorm4_1_6_")
        relu4_1_6_ = mx.symbol.Activation(data=batchnorm4_1_6_,
            act_type='relu',
            name="relu4_1_6_")

        conv5_1_6_ = mx.symbol.Convolution(data=relu4_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_6_")
        # conv5_1_6_, output shape: {[256,56,56]}

        batchnorm5_1_6_ = mx.symbol.BatchNorm(data=conv5_1_6_,
            fix_gamma=True,
            name="batchnorm5_1_6_")
        conv3_1_7_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_7_")
        # conv3_1_7_, output shape: {[4,56,56]}

        batchnorm3_1_7_ = mx.symbol.BatchNorm(data=conv3_1_7_,
            fix_gamma=True,
            name="batchnorm3_1_7_")
        relu3_1_7_ = mx.symbol.Activation(data=batchnorm3_1_7_,
            act_type='relu',
            name="relu3_1_7_")

        conv4_1_7_ = mx.symbol.pad(data=relu3_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_7_ = mx.symbol.Convolution(data=conv4_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_7_")
        # conv4_1_7_, output shape: {[4,56,56]}

        batchnorm4_1_7_ = mx.symbol.BatchNorm(data=conv4_1_7_,
            fix_gamma=True,
            name="batchnorm4_1_7_")
        relu4_1_7_ = mx.symbol.Activation(data=batchnorm4_1_7_,
            act_type='relu',
            name="relu4_1_7_")

        conv5_1_7_ = mx.symbol.Convolution(data=relu4_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_7_")
        # conv5_1_7_, output shape: {[256,56,56]}

        batchnorm5_1_7_ = mx.symbol.BatchNorm(data=conv5_1_7_,
            fix_gamma=True,
            name="batchnorm5_1_7_")
        conv3_1_8_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_8_")
        # conv3_1_8_, output shape: {[4,56,56]}

        batchnorm3_1_8_ = mx.symbol.BatchNorm(data=conv3_1_8_,
            fix_gamma=True,
            name="batchnorm3_1_8_")
        relu3_1_8_ = mx.symbol.Activation(data=batchnorm3_1_8_,
            act_type='relu',
            name="relu3_1_8_")

        conv4_1_8_ = mx.symbol.pad(data=relu3_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_8_ = mx.symbol.Convolution(data=conv4_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_8_")
        # conv4_1_8_, output shape: {[4,56,56]}

        batchnorm4_1_8_ = mx.symbol.BatchNorm(data=conv4_1_8_,
            fix_gamma=True,
            name="batchnorm4_1_8_")
        relu4_1_8_ = mx.symbol.Activation(data=batchnorm4_1_8_,
            act_type='relu',
            name="relu4_1_8_")

        conv5_1_8_ = mx.symbol.Convolution(data=relu4_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_8_")
        # conv5_1_8_, output shape: {[256,56,56]}

        batchnorm5_1_8_ = mx.symbol.BatchNorm(data=conv5_1_8_,
            fix_gamma=True,
            name="batchnorm5_1_8_")
        conv3_1_9_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_9_")
        # conv3_1_9_, output shape: {[4,56,56]}

        batchnorm3_1_9_ = mx.symbol.BatchNorm(data=conv3_1_9_,
            fix_gamma=True,
            name="batchnorm3_1_9_")
        relu3_1_9_ = mx.symbol.Activation(data=batchnorm3_1_9_,
            act_type='relu',
            name="relu3_1_9_")

        conv4_1_9_ = mx.symbol.pad(data=relu3_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_9_ = mx.symbol.Convolution(data=conv4_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_9_")
        # conv4_1_9_, output shape: {[4,56,56]}

        batchnorm4_1_9_ = mx.symbol.BatchNorm(data=conv4_1_9_,
            fix_gamma=True,
            name="batchnorm4_1_9_")
        relu4_1_9_ = mx.symbol.Activation(data=batchnorm4_1_9_,
            act_type='relu',
            name="relu4_1_9_")

        conv5_1_9_ = mx.symbol.Convolution(data=relu4_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_9_")
        # conv5_1_9_, output shape: {[256,56,56]}

        batchnorm5_1_9_ = mx.symbol.BatchNorm(data=conv5_1_9_,
            fix_gamma=True,
            name="batchnorm5_1_9_")
        conv3_1_10_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_10_")
        # conv3_1_10_, output shape: {[4,56,56]}

        batchnorm3_1_10_ = mx.symbol.BatchNorm(data=conv3_1_10_,
            fix_gamma=True,
            name="batchnorm3_1_10_")
        relu3_1_10_ = mx.symbol.Activation(data=batchnorm3_1_10_,
            act_type='relu',
            name="relu3_1_10_")

        conv4_1_10_ = mx.symbol.pad(data=relu3_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_10_ = mx.symbol.Convolution(data=conv4_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_10_")
        # conv4_1_10_, output shape: {[4,56,56]}

        batchnorm4_1_10_ = mx.symbol.BatchNorm(data=conv4_1_10_,
            fix_gamma=True,
            name="batchnorm4_1_10_")
        relu4_1_10_ = mx.symbol.Activation(data=batchnorm4_1_10_,
            act_type='relu',
            name="relu4_1_10_")

        conv5_1_10_ = mx.symbol.Convolution(data=relu4_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_10_")
        # conv5_1_10_, output shape: {[256,56,56]}

        batchnorm5_1_10_ = mx.symbol.BatchNorm(data=conv5_1_10_,
            fix_gamma=True,
            name="batchnorm5_1_10_")
        conv3_1_11_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_11_")
        # conv3_1_11_, output shape: {[4,56,56]}

        batchnorm3_1_11_ = mx.symbol.BatchNorm(data=conv3_1_11_,
            fix_gamma=True,
            name="batchnorm3_1_11_")
        relu3_1_11_ = mx.symbol.Activation(data=batchnorm3_1_11_,
            act_type='relu',
            name="relu3_1_11_")

        conv4_1_11_ = mx.symbol.pad(data=relu3_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_11_ = mx.symbol.Convolution(data=conv4_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_11_")
        # conv4_1_11_, output shape: {[4,56,56]}

        batchnorm4_1_11_ = mx.symbol.BatchNorm(data=conv4_1_11_,
            fix_gamma=True,
            name="batchnorm4_1_11_")
        relu4_1_11_ = mx.symbol.Activation(data=batchnorm4_1_11_,
            act_type='relu',
            name="relu4_1_11_")

        conv5_1_11_ = mx.symbol.Convolution(data=relu4_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_11_")
        # conv5_1_11_, output shape: {[256,56,56]}

        batchnorm5_1_11_ = mx.symbol.BatchNorm(data=conv5_1_11_,
            fix_gamma=True,
            name="batchnorm5_1_11_")
        conv3_1_12_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_12_")
        # conv3_1_12_, output shape: {[4,56,56]}

        batchnorm3_1_12_ = mx.symbol.BatchNorm(data=conv3_1_12_,
            fix_gamma=True,
            name="batchnorm3_1_12_")
        relu3_1_12_ = mx.symbol.Activation(data=batchnorm3_1_12_,
            act_type='relu',
            name="relu3_1_12_")

        conv4_1_12_ = mx.symbol.pad(data=relu3_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_12_ = mx.symbol.Convolution(data=conv4_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_12_")
        # conv4_1_12_, output shape: {[4,56,56]}

        batchnorm4_1_12_ = mx.symbol.BatchNorm(data=conv4_1_12_,
            fix_gamma=True,
            name="batchnorm4_1_12_")
        relu4_1_12_ = mx.symbol.Activation(data=batchnorm4_1_12_,
            act_type='relu',
            name="relu4_1_12_")

        conv5_1_12_ = mx.symbol.Convolution(data=relu4_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_12_")
        # conv5_1_12_, output shape: {[256,56,56]}

        batchnorm5_1_12_ = mx.symbol.BatchNorm(data=conv5_1_12_,
            fix_gamma=True,
            name="batchnorm5_1_12_")
        conv3_1_13_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_13_")
        # conv3_1_13_, output shape: {[4,56,56]}

        batchnorm3_1_13_ = mx.symbol.BatchNorm(data=conv3_1_13_,
            fix_gamma=True,
            name="batchnorm3_1_13_")
        relu3_1_13_ = mx.symbol.Activation(data=batchnorm3_1_13_,
            act_type='relu',
            name="relu3_1_13_")

        conv4_1_13_ = mx.symbol.pad(data=relu3_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_13_ = mx.symbol.Convolution(data=conv4_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_13_")
        # conv4_1_13_, output shape: {[4,56,56]}

        batchnorm4_1_13_ = mx.symbol.BatchNorm(data=conv4_1_13_,
            fix_gamma=True,
            name="batchnorm4_1_13_")
        relu4_1_13_ = mx.symbol.Activation(data=batchnorm4_1_13_,
            act_type='relu',
            name="relu4_1_13_")

        conv5_1_13_ = mx.symbol.Convolution(data=relu4_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_13_")
        # conv5_1_13_, output shape: {[256,56,56]}

        batchnorm5_1_13_ = mx.symbol.BatchNorm(data=conv5_1_13_,
            fix_gamma=True,
            name="batchnorm5_1_13_")
        conv3_1_14_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_14_")
        # conv3_1_14_, output shape: {[4,56,56]}

        batchnorm3_1_14_ = mx.symbol.BatchNorm(data=conv3_1_14_,
            fix_gamma=True,
            name="batchnorm3_1_14_")
        relu3_1_14_ = mx.symbol.Activation(data=batchnorm3_1_14_,
            act_type='relu',
            name="relu3_1_14_")

        conv4_1_14_ = mx.symbol.pad(data=relu3_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_14_ = mx.symbol.Convolution(data=conv4_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_14_")
        # conv4_1_14_, output shape: {[4,56,56]}

        batchnorm4_1_14_ = mx.symbol.BatchNorm(data=conv4_1_14_,
            fix_gamma=True,
            name="batchnorm4_1_14_")
        relu4_1_14_ = mx.symbol.Activation(data=batchnorm4_1_14_,
            act_type='relu',
            name="relu4_1_14_")

        conv5_1_14_ = mx.symbol.Convolution(data=relu4_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_14_")
        # conv5_1_14_, output shape: {[256,56,56]}

        batchnorm5_1_14_ = mx.symbol.BatchNorm(data=conv5_1_14_,
            fix_gamma=True,
            name="batchnorm5_1_14_")
        conv3_1_15_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_15_")
        # conv3_1_15_, output shape: {[4,56,56]}

        batchnorm3_1_15_ = mx.symbol.BatchNorm(data=conv3_1_15_,
            fix_gamma=True,
            name="batchnorm3_1_15_")
        relu3_1_15_ = mx.symbol.Activation(data=batchnorm3_1_15_,
            act_type='relu',
            name="relu3_1_15_")

        conv4_1_15_ = mx.symbol.pad(data=relu3_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_15_ = mx.symbol.Convolution(data=conv4_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_15_")
        # conv4_1_15_, output shape: {[4,56,56]}

        batchnorm4_1_15_ = mx.symbol.BatchNorm(data=conv4_1_15_,
            fix_gamma=True,
            name="batchnorm4_1_15_")
        relu4_1_15_ = mx.symbol.Activation(data=batchnorm4_1_15_,
            act_type='relu',
            name="relu4_1_15_")

        conv5_1_15_ = mx.symbol.Convolution(data=relu4_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_15_")
        # conv5_1_15_, output shape: {[256,56,56]}

        batchnorm5_1_15_ = mx.symbol.BatchNorm(data=conv5_1_15_,
            fix_gamma=True,
            name="batchnorm5_1_15_")
        conv3_1_16_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_16_")
        # conv3_1_16_, output shape: {[4,56,56]}

        batchnorm3_1_16_ = mx.symbol.BatchNorm(data=conv3_1_16_,
            fix_gamma=True,
            name="batchnorm3_1_16_")
        relu3_1_16_ = mx.symbol.Activation(data=batchnorm3_1_16_,
            act_type='relu',
            name="relu3_1_16_")

        conv4_1_16_ = mx.symbol.pad(data=relu3_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_16_ = mx.symbol.Convolution(data=conv4_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_16_")
        # conv4_1_16_, output shape: {[4,56,56]}

        batchnorm4_1_16_ = mx.symbol.BatchNorm(data=conv4_1_16_,
            fix_gamma=True,
            name="batchnorm4_1_16_")
        relu4_1_16_ = mx.symbol.Activation(data=batchnorm4_1_16_,
            act_type='relu',
            name="relu4_1_16_")

        conv5_1_16_ = mx.symbol.Convolution(data=relu4_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_16_")
        # conv5_1_16_, output shape: {[256,56,56]}

        batchnorm5_1_16_ = mx.symbol.BatchNorm(data=conv5_1_16_,
            fix_gamma=True,
            name="batchnorm5_1_16_")
        conv3_1_17_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_17_")
        # conv3_1_17_, output shape: {[4,56,56]}

        batchnorm3_1_17_ = mx.symbol.BatchNorm(data=conv3_1_17_,
            fix_gamma=True,
            name="batchnorm3_1_17_")
        relu3_1_17_ = mx.symbol.Activation(data=batchnorm3_1_17_,
            act_type='relu',
            name="relu3_1_17_")

        conv4_1_17_ = mx.symbol.pad(data=relu3_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_17_ = mx.symbol.Convolution(data=conv4_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_17_")
        # conv4_1_17_, output shape: {[4,56,56]}

        batchnorm4_1_17_ = mx.symbol.BatchNorm(data=conv4_1_17_,
            fix_gamma=True,
            name="batchnorm4_1_17_")
        relu4_1_17_ = mx.symbol.Activation(data=batchnorm4_1_17_,
            act_type='relu',
            name="relu4_1_17_")

        conv5_1_17_ = mx.symbol.Convolution(data=relu4_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_17_")
        # conv5_1_17_, output shape: {[256,56,56]}

        batchnorm5_1_17_ = mx.symbol.BatchNorm(data=conv5_1_17_,
            fix_gamma=True,
            name="batchnorm5_1_17_")
        conv3_1_18_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_18_")
        # conv3_1_18_, output shape: {[4,56,56]}

        batchnorm3_1_18_ = mx.symbol.BatchNorm(data=conv3_1_18_,
            fix_gamma=True,
            name="batchnorm3_1_18_")
        relu3_1_18_ = mx.symbol.Activation(data=batchnorm3_1_18_,
            act_type='relu',
            name="relu3_1_18_")

        conv4_1_18_ = mx.symbol.pad(data=relu3_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_18_ = mx.symbol.Convolution(data=conv4_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_18_")
        # conv4_1_18_, output shape: {[4,56,56]}

        batchnorm4_1_18_ = mx.symbol.BatchNorm(data=conv4_1_18_,
            fix_gamma=True,
            name="batchnorm4_1_18_")
        relu4_1_18_ = mx.symbol.Activation(data=batchnorm4_1_18_,
            act_type='relu',
            name="relu4_1_18_")

        conv5_1_18_ = mx.symbol.Convolution(data=relu4_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_18_")
        # conv5_1_18_, output shape: {[256,56,56]}

        batchnorm5_1_18_ = mx.symbol.BatchNorm(data=conv5_1_18_,
            fix_gamma=True,
            name="batchnorm5_1_18_")
        conv3_1_19_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_19_")
        # conv3_1_19_, output shape: {[4,56,56]}

        batchnorm3_1_19_ = mx.symbol.BatchNorm(data=conv3_1_19_,
            fix_gamma=True,
            name="batchnorm3_1_19_")
        relu3_1_19_ = mx.symbol.Activation(data=batchnorm3_1_19_,
            act_type='relu',
            name="relu3_1_19_")

        conv4_1_19_ = mx.symbol.pad(data=relu3_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_19_ = mx.symbol.Convolution(data=conv4_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_19_")
        # conv4_1_19_, output shape: {[4,56,56]}

        batchnorm4_1_19_ = mx.symbol.BatchNorm(data=conv4_1_19_,
            fix_gamma=True,
            name="batchnorm4_1_19_")
        relu4_1_19_ = mx.symbol.Activation(data=batchnorm4_1_19_,
            act_type='relu',
            name="relu4_1_19_")

        conv5_1_19_ = mx.symbol.Convolution(data=relu4_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_19_")
        # conv5_1_19_, output shape: {[256,56,56]}

        batchnorm5_1_19_ = mx.symbol.BatchNorm(data=conv5_1_19_,
            fix_gamma=True,
            name="batchnorm5_1_19_")
        conv3_1_20_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_20_")
        # conv3_1_20_, output shape: {[4,56,56]}

        batchnorm3_1_20_ = mx.symbol.BatchNorm(data=conv3_1_20_,
            fix_gamma=True,
            name="batchnorm3_1_20_")
        relu3_1_20_ = mx.symbol.Activation(data=batchnorm3_1_20_,
            act_type='relu',
            name="relu3_1_20_")

        conv4_1_20_ = mx.symbol.pad(data=relu3_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_20_ = mx.symbol.Convolution(data=conv4_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_20_")
        # conv4_1_20_, output shape: {[4,56,56]}

        batchnorm4_1_20_ = mx.symbol.BatchNorm(data=conv4_1_20_,
            fix_gamma=True,
            name="batchnorm4_1_20_")
        relu4_1_20_ = mx.symbol.Activation(data=batchnorm4_1_20_,
            act_type='relu',
            name="relu4_1_20_")

        conv5_1_20_ = mx.symbol.Convolution(data=relu4_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_20_")
        # conv5_1_20_, output shape: {[256,56,56]}

        batchnorm5_1_20_ = mx.symbol.BatchNorm(data=conv5_1_20_,
            fix_gamma=True,
            name="batchnorm5_1_20_")
        conv3_1_21_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_21_")
        # conv3_1_21_, output shape: {[4,56,56]}

        batchnorm3_1_21_ = mx.symbol.BatchNorm(data=conv3_1_21_,
            fix_gamma=True,
            name="batchnorm3_1_21_")
        relu3_1_21_ = mx.symbol.Activation(data=batchnorm3_1_21_,
            act_type='relu',
            name="relu3_1_21_")

        conv4_1_21_ = mx.symbol.pad(data=relu3_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_21_ = mx.symbol.Convolution(data=conv4_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_21_")
        # conv4_1_21_, output shape: {[4,56,56]}

        batchnorm4_1_21_ = mx.symbol.BatchNorm(data=conv4_1_21_,
            fix_gamma=True,
            name="batchnorm4_1_21_")
        relu4_1_21_ = mx.symbol.Activation(data=batchnorm4_1_21_,
            act_type='relu',
            name="relu4_1_21_")

        conv5_1_21_ = mx.symbol.Convolution(data=relu4_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_21_")
        # conv5_1_21_, output shape: {[256,56,56]}

        batchnorm5_1_21_ = mx.symbol.BatchNorm(data=conv5_1_21_,
            fix_gamma=True,
            name="batchnorm5_1_21_")
        conv3_1_22_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_22_")
        # conv3_1_22_, output shape: {[4,56,56]}

        batchnorm3_1_22_ = mx.symbol.BatchNorm(data=conv3_1_22_,
            fix_gamma=True,
            name="batchnorm3_1_22_")
        relu3_1_22_ = mx.symbol.Activation(data=batchnorm3_1_22_,
            act_type='relu',
            name="relu3_1_22_")

        conv4_1_22_ = mx.symbol.pad(data=relu3_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_22_ = mx.symbol.Convolution(data=conv4_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_22_")
        # conv4_1_22_, output shape: {[4,56,56]}

        batchnorm4_1_22_ = mx.symbol.BatchNorm(data=conv4_1_22_,
            fix_gamma=True,
            name="batchnorm4_1_22_")
        relu4_1_22_ = mx.symbol.Activation(data=batchnorm4_1_22_,
            act_type='relu',
            name="relu4_1_22_")

        conv5_1_22_ = mx.symbol.Convolution(data=relu4_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_22_")
        # conv5_1_22_, output shape: {[256,56,56]}

        batchnorm5_1_22_ = mx.symbol.BatchNorm(data=conv5_1_22_,
            fix_gamma=True,
            name="batchnorm5_1_22_")
        conv3_1_23_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_23_")
        # conv3_1_23_, output shape: {[4,56,56]}

        batchnorm3_1_23_ = mx.symbol.BatchNorm(data=conv3_1_23_,
            fix_gamma=True,
            name="batchnorm3_1_23_")
        relu3_1_23_ = mx.symbol.Activation(data=batchnorm3_1_23_,
            act_type='relu',
            name="relu3_1_23_")

        conv4_1_23_ = mx.symbol.pad(data=relu3_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_23_ = mx.symbol.Convolution(data=conv4_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_23_")
        # conv4_1_23_, output shape: {[4,56,56]}

        batchnorm4_1_23_ = mx.symbol.BatchNorm(data=conv4_1_23_,
            fix_gamma=True,
            name="batchnorm4_1_23_")
        relu4_1_23_ = mx.symbol.Activation(data=batchnorm4_1_23_,
            act_type='relu',
            name="relu4_1_23_")

        conv5_1_23_ = mx.symbol.Convolution(data=relu4_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_23_")
        # conv5_1_23_, output shape: {[256,56,56]}

        batchnorm5_1_23_ = mx.symbol.BatchNorm(data=conv5_1_23_,
            fix_gamma=True,
            name="batchnorm5_1_23_")
        conv3_1_24_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_24_")
        # conv3_1_24_, output shape: {[4,56,56]}

        batchnorm3_1_24_ = mx.symbol.BatchNorm(data=conv3_1_24_,
            fix_gamma=True,
            name="batchnorm3_1_24_")
        relu3_1_24_ = mx.symbol.Activation(data=batchnorm3_1_24_,
            act_type='relu',
            name="relu3_1_24_")

        conv4_1_24_ = mx.symbol.pad(data=relu3_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_24_ = mx.symbol.Convolution(data=conv4_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_24_")
        # conv4_1_24_, output shape: {[4,56,56]}

        batchnorm4_1_24_ = mx.symbol.BatchNorm(data=conv4_1_24_,
            fix_gamma=True,
            name="batchnorm4_1_24_")
        relu4_1_24_ = mx.symbol.Activation(data=batchnorm4_1_24_,
            act_type='relu',
            name="relu4_1_24_")

        conv5_1_24_ = mx.symbol.Convolution(data=relu4_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_24_")
        # conv5_1_24_, output shape: {[256,56,56]}

        batchnorm5_1_24_ = mx.symbol.BatchNorm(data=conv5_1_24_,
            fix_gamma=True,
            name="batchnorm5_1_24_")
        conv3_1_25_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_25_")
        # conv3_1_25_, output shape: {[4,56,56]}

        batchnorm3_1_25_ = mx.symbol.BatchNorm(data=conv3_1_25_,
            fix_gamma=True,
            name="batchnorm3_1_25_")
        relu3_1_25_ = mx.symbol.Activation(data=batchnorm3_1_25_,
            act_type='relu',
            name="relu3_1_25_")

        conv4_1_25_ = mx.symbol.pad(data=relu3_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_25_ = mx.symbol.Convolution(data=conv4_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_25_")
        # conv4_1_25_, output shape: {[4,56,56]}

        batchnorm4_1_25_ = mx.symbol.BatchNorm(data=conv4_1_25_,
            fix_gamma=True,
            name="batchnorm4_1_25_")
        relu4_1_25_ = mx.symbol.Activation(data=batchnorm4_1_25_,
            act_type='relu',
            name="relu4_1_25_")

        conv5_1_25_ = mx.symbol.Convolution(data=relu4_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_25_")
        # conv5_1_25_, output shape: {[256,56,56]}

        batchnorm5_1_25_ = mx.symbol.BatchNorm(data=conv5_1_25_,
            fix_gamma=True,
            name="batchnorm5_1_25_")
        conv3_1_26_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_26_")
        # conv3_1_26_, output shape: {[4,56,56]}

        batchnorm3_1_26_ = mx.symbol.BatchNorm(data=conv3_1_26_,
            fix_gamma=True,
            name="batchnorm3_1_26_")
        relu3_1_26_ = mx.symbol.Activation(data=batchnorm3_1_26_,
            act_type='relu',
            name="relu3_1_26_")

        conv4_1_26_ = mx.symbol.pad(data=relu3_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_26_ = mx.symbol.Convolution(data=conv4_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_26_")
        # conv4_1_26_, output shape: {[4,56,56]}

        batchnorm4_1_26_ = mx.symbol.BatchNorm(data=conv4_1_26_,
            fix_gamma=True,
            name="batchnorm4_1_26_")
        relu4_1_26_ = mx.symbol.Activation(data=batchnorm4_1_26_,
            act_type='relu',
            name="relu4_1_26_")

        conv5_1_26_ = mx.symbol.Convolution(data=relu4_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_26_")
        # conv5_1_26_, output shape: {[256,56,56]}

        batchnorm5_1_26_ = mx.symbol.BatchNorm(data=conv5_1_26_,
            fix_gamma=True,
            name="batchnorm5_1_26_")
        conv3_1_27_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_27_")
        # conv3_1_27_, output shape: {[4,56,56]}

        batchnorm3_1_27_ = mx.symbol.BatchNorm(data=conv3_1_27_,
            fix_gamma=True,
            name="batchnorm3_1_27_")
        relu3_1_27_ = mx.symbol.Activation(data=batchnorm3_1_27_,
            act_type='relu',
            name="relu3_1_27_")

        conv4_1_27_ = mx.symbol.pad(data=relu3_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_27_ = mx.symbol.Convolution(data=conv4_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_27_")
        # conv4_1_27_, output shape: {[4,56,56]}

        batchnorm4_1_27_ = mx.symbol.BatchNorm(data=conv4_1_27_,
            fix_gamma=True,
            name="batchnorm4_1_27_")
        relu4_1_27_ = mx.symbol.Activation(data=batchnorm4_1_27_,
            act_type='relu',
            name="relu4_1_27_")

        conv5_1_27_ = mx.symbol.Convolution(data=relu4_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_27_")
        # conv5_1_27_, output shape: {[256,56,56]}

        batchnorm5_1_27_ = mx.symbol.BatchNorm(data=conv5_1_27_,
            fix_gamma=True,
            name="batchnorm5_1_27_")
        conv3_1_28_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_28_")
        # conv3_1_28_, output shape: {[4,56,56]}

        batchnorm3_1_28_ = mx.symbol.BatchNorm(data=conv3_1_28_,
            fix_gamma=True,
            name="batchnorm3_1_28_")
        relu3_1_28_ = mx.symbol.Activation(data=batchnorm3_1_28_,
            act_type='relu',
            name="relu3_1_28_")

        conv4_1_28_ = mx.symbol.pad(data=relu3_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_28_ = mx.symbol.Convolution(data=conv4_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_28_")
        # conv4_1_28_, output shape: {[4,56,56]}

        batchnorm4_1_28_ = mx.symbol.BatchNorm(data=conv4_1_28_,
            fix_gamma=True,
            name="batchnorm4_1_28_")
        relu4_1_28_ = mx.symbol.Activation(data=batchnorm4_1_28_,
            act_type='relu',
            name="relu4_1_28_")

        conv5_1_28_ = mx.symbol.Convolution(data=relu4_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_28_")
        # conv5_1_28_, output shape: {[256,56,56]}

        batchnorm5_1_28_ = mx.symbol.BatchNorm(data=conv5_1_28_,
            fix_gamma=True,
            name="batchnorm5_1_28_")
        conv3_1_29_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_29_")
        # conv3_1_29_, output shape: {[4,56,56]}

        batchnorm3_1_29_ = mx.symbol.BatchNorm(data=conv3_1_29_,
            fix_gamma=True,
            name="batchnorm3_1_29_")
        relu3_1_29_ = mx.symbol.Activation(data=batchnorm3_1_29_,
            act_type='relu',
            name="relu3_1_29_")

        conv4_1_29_ = mx.symbol.pad(data=relu3_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_29_ = mx.symbol.Convolution(data=conv4_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_29_")
        # conv4_1_29_, output shape: {[4,56,56]}

        batchnorm4_1_29_ = mx.symbol.BatchNorm(data=conv4_1_29_,
            fix_gamma=True,
            name="batchnorm4_1_29_")
        relu4_1_29_ = mx.symbol.Activation(data=batchnorm4_1_29_,
            act_type='relu',
            name="relu4_1_29_")

        conv5_1_29_ = mx.symbol.Convolution(data=relu4_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_29_")
        # conv5_1_29_, output shape: {[256,56,56]}

        batchnorm5_1_29_ = mx.symbol.BatchNorm(data=conv5_1_29_,
            fix_gamma=True,
            name="batchnorm5_1_29_")
        conv3_1_30_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_30_")
        # conv3_1_30_, output shape: {[4,56,56]}

        batchnorm3_1_30_ = mx.symbol.BatchNorm(data=conv3_1_30_,
            fix_gamma=True,
            name="batchnorm3_1_30_")
        relu3_1_30_ = mx.symbol.Activation(data=batchnorm3_1_30_,
            act_type='relu',
            name="relu3_1_30_")

        conv4_1_30_ = mx.symbol.pad(data=relu3_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_30_ = mx.symbol.Convolution(data=conv4_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_30_")
        # conv4_1_30_, output shape: {[4,56,56]}

        batchnorm4_1_30_ = mx.symbol.BatchNorm(data=conv4_1_30_,
            fix_gamma=True,
            name="batchnorm4_1_30_")
        relu4_1_30_ = mx.symbol.Activation(data=batchnorm4_1_30_,
            act_type='relu',
            name="relu4_1_30_")

        conv5_1_30_ = mx.symbol.Convolution(data=relu4_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_30_")
        # conv5_1_30_, output shape: {[256,56,56]}

        batchnorm5_1_30_ = mx.symbol.BatchNorm(data=conv5_1_30_,
            fix_gamma=True,
            name="batchnorm5_1_30_")
        conv3_1_31_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_31_")
        # conv3_1_31_, output shape: {[4,56,56]}

        batchnorm3_1_31_ = mx.symbol.BatchNorm(data=conv3_1_31_,
            fix_gamma=True,
            name="batchnorm3_1_31_")
        relu3_1_31_ = mx.symbol.Activation(data=batchnorm3_1_31_,
            act_type='relu',
            name="relu3_1_31_")

        conv4_1_31_ = mx.symbol.pad(data=relu3_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_31_ = mx.symbol.Convolution(data=conv4_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_31_")
        # conv4_1_31_, output shape: {[4,56,56]}

        batchnorm4_1_31_ = mx.symbol.BatchNorm(data=conv4_1_31_,
            fix_gamma=True,
            name="batchnorm4_1_31_")
        relu4_1_31_ = mx.symbol.Activation(data=batchnorm4_1_31_,
            act_type='relu',
            name="relu4_1_31_")

        conv5_1_31_ = mx.symbol.Convolution(data=relu4_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_31_")
        # conv5_1_31_, output shape: {[256,56,56]}

        batchnorm5_1_31_ = mx.symbol.BatchNorm(data=conv5_1_31_,
            fix_gamma=True,
            name="batchnorm5_1_31_")
        conv3_1_32_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv3_1_32_")
        # conv3_1_32_, output shape: {[4,56,56]}

        batchnorm3_1_32_ = mx.symbol.BatchNorm(data=conv3_1_32_,
            fix_gamma=True,
            name="batchnorm3_1_32_")
        relu3_1_32_ = mx.symbol.Activation(data=batchnorm3_1_32_,
            act_type='relu',
            name="relu3_1_32_")

        conv4_1_32_ = mx.symbol.pad(data=relu3_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_1_32_ = mx.symbol.Convolution(data=conv4_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv4_1_32_")
        # conv4_1_32_, output shape: {[4,56,56]}

        batchnorm4_1_32_ = mx.symbol.BatchNorm(data=conv4_1_32_,
            fix_gamma=True,
            name="batchnorm4_1_32_")
        relu4_1_32_ = mx.symbol.Activation(data=batchnorm4_1_32_,
            act_type='relu',
            name="relu4_1_32_")

        conv5_1_32_ = mx.symbol.Convolution(data=relu4_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_1_32_")
        # conv5_1_32_, output shape: {[256,56,56]}

        batchnorm5_1_32_ = mx.symbol.BatchNorm(data=conv5_1_32_,
            fix_gamma=True,
            name="batchnorm5_1_32_")
        add6_1_ = batchnorm5_1_1_ + batchnorm5_1_2_ + batchnorm5_1_3_ + batchnorm5_1_4_ + batchnorm5_1_5_ + batchnorm5_1_6_ + batchnorm5_1_7_ + batchnorm5_1_8_ + batchnorm5_1_9_ + batchnorm5_1_10_ + batchnorm5_1_11_ + batchnorm5_1_12_ + batchnorm5_1_13_ + batchnorm5_1_14_ + batchnorm5_1_15_ + batchnorm5_1_16_ + batchnorm5_1_17_ + batchnorm5_1_18_ + batchnorm5_1_19_ + batchnorm5_1_20_ + batchnorm5_1_21_ + batchnorm5_1_22_ + batchnorm5_1_23_ + batchnorm5_1_24_ + batchnorm5_1_25_ + batchnorm5_1_26_ + batchnorm5_1_27_ + batchnorm5_1_28_ + batchnorm5_1_29_ + batchnorm5_1_30_ + batchnorm5_1_31_ + batchnorm5_1_32_
        # add6_1_, output shape: {[256,56,56]}

        conv2_2_ = mx.symbol.Convolution(data=pool1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv2_2_")
        # conv2_2_, output shape: {[256,56,56]}

        batchnorm2_2_ = mx.symbol.BatchNorm(data=conv2_2_,
            fix_gamma=True,
            name="batchnorm2_2_")
        add7_ = add6_1_ + batchnorm2_2_
        # add7_, output shape: {[256,56,56]}

        relu7_ = mx.symbol.Activation(data=add7_,
            act_type='relu',
            name="relu7_")

        conv9_1_1_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_1_")
        # conv9_1_1_, output shape: {[4,56,56]}

        batchnorm9_1_1_ = mx.symbol.BatchNorm(data=conv9_1_1_,
            fix_gamma=True,
            name="batchnorm9_1_1_")
        relu9_1_1_ = mx.symbol.Activation(data=batchnorm9_1_1_,
            act_type='relu',
            name="relu9_1_1_")

        conv10_1_1_ = mx.symbol.pad(data=relu9_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_1_ = mx.symbol.Convolution(data=conv10_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_1_")
        # conv10_1_1_, output shape: {[4,56,56]}

        batchnorm10_1_1_ = mx.symbol.BatchNorm(data=conv10_1_1_,
            fix_gamma=True,
            name="batchnorm10_1_1_")
        relu10_1_1_ = mx.symbol.Activation(data=batchnorm10_1_1_,
            act_type='relu',
            name="relu10_1_1_")

        conv11_1_1_ = mx.symbol.Convolution(data=relu10_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_1_")
        # conv11_1_1_, output shape: {[256,56,56]}

        batchnorm11_1_1_ = mx.symbol.BatchNorm(data=conv11_1_1_,
            fix_gamma=True,
            name="batchnorm11_1_1_")
        conv9_1_2_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_2_")
        # conv9_1_2_, output shape: {[4,56,56]}

        batchnorm9_1_2_ = mx.symbol.BatchNorm(data=conv9_1_2_,
            fix_gamma=True,
            name="batchnorm9_1_2_")
        relu9_1_2_ = mx.symbol.Activation(data=batchnorm9_1_2_,
            act_type='relu',
            name="relu9_1_2_")

        conv10_1_2_ = mx.symbol.pad(data=relu9_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_2_ = mx.symbol.Convolution(data=conv10_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_2_")
        # conv10_1_2_, output shape: {[4,56,56]}

        batchnorm10_1_2_ = mx.symbol.BatchNorm(data=conv10_1_2_,
            fix_gamma=True,
            name="batchnorm10_1_2_")
        relu10_1_2_ = mx.symbol.Activation(data=batchnorm10_1_2_,
            act_type='relu',
            name="relu10_1_2_")

        conv11_1_2_ = mx.symbol.Convolution(data=relu10_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_2_")
        # conv11_1_2_, output shape: {[256,56,56]}

        batchnorm11_1_2_ = mx.symbol.BatchNorm(data=conv11_1_2_,
            fix_gamma=True,
            name="batchnorm11_1_2_")
        conv9_1_3_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_3_")
        # conv9_1_3_, output shape: {[4,56,56]}

        batchnorm9_1_3_ = mx.symbol.BatchNorm(data=conv9_1_3_,
            fix_gamma=True,
            name="batchnorm9_1_3_")
        relu9_1_3_ = mx.symbol.Activation(data=batchnorm9_1_3_,
            act_type='relu',
            name="relu9_1_3_")

        conv10_1_3_ = mx.symbol.pad(data=relu9_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_3_ = mx.symbol.Convolution(data=conv10_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_3_")
        # conv10_1_3_, output shape: {[4,56,56]}

        batchnorm10_1_3_ = mx.symbol.BatchNorm(data=conv10_1_3_,
            fix_gamma=True,
            name="batchnorm10_1_3_")
        relu10_1_3_ = mx.symbol.Activation(data=batchnorm10_1_3_,
            act_type='relu',
            name="relu10_1_3_")

        conv11_1_3_ = mx.symbol.Convolution(data=relu10_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_3_")
        # conv11_1_3_, output shape: {[256,56,56]}

        batchnorm11_1_3_ = mx.symbol.BatchNorm(data=conv11_1_3_,
            fix_gamma=True,
            name="batchnorm11_1_3_")
        conv9_1_4_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_4_")
        # conv9_1_4_, output shape: {[4,56,56]}

        batchnorm9_1_4_ = mx.symbol.BatchNorm(data=conv9_1_4_,
            fix_gamma=True,
            name="batchnorm9_1_4_")
        relu9_1_4_ = mx.symbol.Activation(data=batchnorm9_1_4_,
            act_type='relu',
            name="relu9_1_4_")

        conv10_1_4_ = mx.symbol.pad(data=relu9_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_4_ = mx.symbol.Convolution(data=conv10_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_4_")
        # conv10_1_4_, output shape: {[4,56,56]}

        batchnorm10_1_4_ = mx.symbol.BatchNorm(data=conv10_1_4_,
            fix_gamma=True,
            name="batchnorm10_1_4_")
        relu10_1_4_ = mx.symbol.Activation(data=batchnorm10_1_4_,
            act_type='relu',
            name="relu10_1_4_")

        conv11_1_4_ = mx.symbol.Convolution(data=relu10_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_4_")
        # conv11_1_4_, output shape: {[256,56,56]}

        batchnorm11_1_4_ = mx.symbol.BatchNorm(data=conv11_1_4_,
            fix_gamma=True,
            name="batchnorm11_1_4_")
        conv9_1_5_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_5_")
        # conv9_1_5_, output shape: {[4,56,56]}

        batchnorm9_1_5_ = mx.symbol.BatchNorm(data=conv9_1_5_,
            fix_gamma=True,
            name="batchnorm9_1_5_")
        relu9_1_5_ = mx.symbol.Activation(data=batchnorm9_1_5_,
            act_type='relu',
            name="relu9_1_5_")

        conv10_1_5_ = mx.symbol.pad(data=relu9_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_5_ = mx.symbol.Convolution(data=conv10_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_5_")
        # conv10_1_5_, output shape: {[4,56,56]}

        batchnorm10_1_5_ = mx.symbol.BatchNorm(data=conv10_1_5_,
            fix_gamma=True,
            name="batchnorm10_1_5_")
        relu10_1_5_ = mx.symbol.Activation(data=batchnorm10_1_5_,
            act_type='relu',
            name="relu10_1_5_")

        conv11_1_5_ = mx.symbol.Convolution(data=relu10_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_5_")
        # conv11_1_5_, output shape: {[256,56,56]}

        batchnorm11_1_5_ = mx.symbol.BatchNorm(data=conv11_1_5_,
            fix_gamma=True,
            name="batchnorm11_1_5_")
        conv9_1_6_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_6_")
        # conv9_1_6_, output shape: {[4,56,56]}

        batchnorm9_1_6_ = mx.symbol.BatchNorm(data=conv9_1_6_,
            fix_gamma=True,
            name="batchnorm9_1_6_")
        relu9_1_6_ = mx.symbol.Activation(data=batchnorm9_1_6_,
            act_type='relu',
            name="relu9_1_6_")

        conv10_1_6_ = mx.symbol.pad(data=relu9_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_6_ = mx.symbol.Convolution(data=conv10_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_6_")
        # conv10_1_6_, output shape: {[4,56,56]}

        batchnorm10_1_6_ = mx.symbol.BatchNorm(data=conv10_1_6_,
            fix_gamma=True,
            name="batchnorm10_1_6_")
        relu10_1_6_ = mx.symbol.Activation(data=batchnorm10_1_6_,
            act_type='relu',
            name="relu10_1_6_")

        conv11_1_6_ = mx.symbol.Convolution(data=relu10_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_6_")
        # conv11_1_6_, output shape: {[256,56,56]}

        batchnorm11_1_6_ = mx.symbol.BatchNorm(data=conv11_1_6_,
            fix_gamma=True,
            name="batchnorm11_1_6_")
        conv9_1_7_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_7_")
        # conv9_1_7_, output shape: {[4,56,56]}

        batchnorm9_1_7_ = mx.symbol.BatchNorm(data=conv9_1_7_,
            fix_gamma=True,
            name="batchnorm9_1_7_")
        relu9_1_7_ = mx.symbol.Activation(data=batchnorm9_1_7_,
            act_type='relu',
            name="relu9_1_7_")

        conv10_1_7_ = mx.symbol.pad(data=relu9_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_7_ = mx.symbol.Convolution(data=conv10_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_7_")
        # conv10_1_7_, output shape: {[4,56,56]}

        batchnorm10_1_7_ = mx.symbol.BatchNorm(data=conv10_1_7_,
            fix_gamma=True,
            name="batchnorm10_1_7_")
        relu10_1_7_ = mx.symbol.Activation(data=batchnorm10_1_7_,
            act_type='relu',
            name="relu10_1_7_")

        conv11_1_7_ = mx.symbol.Convolution(data=relu10_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_7_")
        # conv11_1_7_, output shape: {[256,56,56]}

        batchnorm11_1_7_ = mx.symbol.BatchNorm(data=conv11_1_7_,
            fix_gamma=True,
            name="batchnorm11_1_7_")
        conv9_1_8_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_8_")
        # conv9_1_8_, output shape: {[4,56,56]}

        batchnorm9_1_8_ = mx.symbol.BatchNorm(data=conv9_1_8_,
            fix_gamma=True,
            name="batchnorm9_1_8_")
        relu9_1_8_ = mx.symbol.Activation(data=batchnorm9_1_8_,
            act_type='relu',
            name="relu9_1_8_")

        conv10_1_8_ = mx.symbol.pad(data=relu9_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_8_ = mx.symbol.Convolution(data=conv10_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_8_")
        # conv10_1_8_, output shape: {[4,56,56]}

        batchnorm10_1_8_ = mx.symbol.BatchNorm(data=conv10_1_8_,
            fix_gamma=True,
            name="batchnorm10_1_8_")
        relu10_1_8_ = mx.symbol.Activation(data=batchnorm10_1_8_,
            act_type='relu',
            name="relu10_1_8_")

        conv11_1_8_ = mx.symbol.Convolution(data=relu10_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_8_")
        # conv11_1_8_, output shape: {[256,56,56]}

        batchnorm11_1_8_ = mx.symbol.BatchNorm(data=conv11_1_8_,
            fix_gamma=True,
            name="batchnorm11_1_8_")
        conv9_1_9_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_9_")
        # conv9_1_9_, output shape: {[4,56,56]}

        batchnorm9_1_9_ = mx.symbol.BatchNorm(data=conv9_1_9_,
            fix_gamma=True,
            name="batchnorm9_1_9_")
        relu9_1_9_ = mx.symbol.Activation(data=batchnorm9_1_9_,
            act_type='relu',
            name="relu9_1_9_")

        conv10_1_9_ = mx.symbol.pad(data=relu9_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_9_ = mx.symbol.Convolution(data=conv10_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_9_")
        # conv10_1_9_, output shape: {[4,56,56]}

        batchnorm10_1_9_ = mx.symbol.BatchNorm(data=conv10_1_9_,
            fix_gamma=True,
            name="batchnorm10_1_9_")
        relu10_1_9_ = mx.symbol.Activation(data=batchnorm10_1_9_,
            act_type='relu',
            name="relu10_1_9_")

        conv11_1_9_ = mx.symbol.Convolution(data=relu10_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_9_")
        # conv11_1_9_, output shape: {[256,56,56]}

        batchnorm11_1_9_ = mx.symbol.BatchNorm(data=conv11_1_9_,
            fix_gamma=True,
            name="batchnorm11_1_9_")
        conv9_1_10_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_10_")
        # conv9_1_10_, output shape: {[4,56,56]}

        batchnorm9_1_10_ = mx.symbol.BatchNorm(data=conv9_1_10_,
            fix_gamma=True,
            name="batchnorm9_1_10_")
        relu9_1_10_ = mx.symbol.Activation(data=batchnorm9_1_10_,
            act_type='relu',
            name="relu9_1_10_")

        conv10_1_10_ = mx.symbol.pad(data=relu9_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_10_ = mx.symbol.Convolution(data=conv10_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_10_")
        # conv10_1_10_, output shape: {[4,56,56]}

        batchnorm10_1_10_ = mx.symbol.BatchNorm(data=conv10_1_10_,
            fix_gamma=True,
            name="batchnorm10_1_10_")
        relu10_1_10_ = mx.symbol.Activation(data=batchnorm10_1_10_,
            act_type='relu',
            name="relu10_1_10_")

        conv11_1_10_ = mx.symbol.Convolution(data=relu10_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_10_")
        # conv11_1_10_, output shape: {[256,56,56]}

        batchnorm11_1_10_ = mx.symbol.BatchNorm(data=conv11_1_10_,
            fix_gamma=True,
            name="batchnorm11_1_10_")
        conv9_1_11_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_11_")
        # conv9_1_11_, output shape: {[4,56,56]}

        batchnorm9_1_11_ = mx.symbol.BatchNorm(data=conv9_1_11_,
            fix_gamma=True,
            name="batchnorm9_1_11_")
        relu9_1_11_ = mx.symbol.Activation(data=batchnorm9_1_11_,
            act_type='relu',
            name="relu9_1_11_")

        conv10_1_11_ = mx.symbol.pad(data=relu9_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_11_ = mx.symbol.Convolution(data=conv10_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_11_")
        # conv10_1_11_, output shape: {[4,56,56]}

        batchnorm10_1_11_ = mx.symbol.BatchNorm(data=conv10_1_11_,
            fix_gamma=True,
            name="batchnorm10_1_11_")
        relu10_1_11_ = mx.symbol.Activation(data=batchnorm10_1_11_,
            act_type='relu',
            name="relu10_1_11_")

        conv11_1_11_ = mx.symbol.Convolution(data=relu10_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_11_")
        # conv11_1_11_, output shape: {[256,56,56]}

        batchnorm11_1_11_ = mx.symbol.BatchNorm(data=conv11_1_11_,
            fix_gamma=True,
            name="batchnorm11_1_11_")
        conv9_1_12_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_12_")
        # conv9_1_12_, output shape: {[4,56,56]}

        batchnorm9_1_12_ = mx.symbol.BatchNorm(data=conv9_1_12_,
            fix_gamma=True,
            name="batchnorm9_1_12_")
        relu9_1_12_ = mx.symbol.Activation(data=batchnorm9_1_12_,
            act_type='relu',
            name="relu9_1_12_")

        conv10_1_12_ = mx.symbol.pad(data=relu9_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_12_ = mx.symbol.Convolution(data=conv10_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_12_")
        # conv10_1_12_, output shape: {[4,56,56]}

        batchnorm10_1_12_ = mx.symbol.BatchNorm(data=conv10_1_12_,
            fix_gamma=True,
            name="batchnorm10_1_12_")
        relu10_1_12_ = mx.symbol.Activation(data=batchnorm10_1_12_,
            act_type='relu',
            name="relu10_1_12_")

        conv11_1_12_ = mx.symbol.Convolution(data=relu10_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_12_")
        # conv11_1_12_, output shape: {[256,56,56]}

        batchnorm11_1_12_ = mx.symbol.BatchNorm(data=conv11_1_12_,
            fix_gamma=True,
            name="batchnorm11_1_12_")
        conv9_1_13_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_13_")
        # conv9_1_13_, output shape: {[4,56,56]}

        batchnorm9_1_13_ = mx.symbol.BatchNorm(data=conv9_1_13_,
            fix_gamma=True,
            name="batchnorm9_1_13_")
        relu9_1_13_ = mx.symbol.Activation(data=batchnorm9_1_13_,
            act_type='relu',
            name="relu9_1_13_")

        conv10_1_13_ = mx.symbol.pad(data=relu9_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_13_ = mx.symbol.Convolution(data=conv10_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_13_")
        # conv10_1_13_, output shape: {[4,56,56]}

        batchnorm10_1_13_ = mx.symbol.BatchNorm(data=conv10_1_13_,
            fix_gamma=True,
            name="batchnorm10_1_13_")
        relu10_1_13_ = mx.symbol.Activation(data=batchnorm10_1_13_,
            act_type='relu',
            name="relu10_1_13_")

        conv11_1_13_ = mx.symbol.Convolution(data=relu10_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_13_")
        # conv11_1_13_, output shape: {[256,56,56]}

        batchnorm11_1_13_ = mx.symbol.BatchNorm(data=conv11_1_13_,
            fix_gamma=True,
            name="batchnorm11_1_13_")
        conv9_1_14_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_14_")
        # conv9_1_14_, output shape: {[4,56,56]}

        batchnorm9_1_14_ = mx.symbol.BatchNorm(data=conv9_1_14_,
            fix_gamma=True,
            name="batchnorm9_1_14_")
        relu9_1_14_ = mx.symbol.Activation(data=batchnorm9_1_14_,
            act_type='relu',
            name="relu9_1_14_")

        conv10_1_14_ = mx.symbol.pad(data=relu9_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_14_ = mx.symbol.Convolution(data=conv10_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_14_")
        # conv10_1_14_, output shape: {[4,56,56]}

        batchnorm10_1_14_ = mx.symbol.BatchNorm(data=conv10_1_14_,
            fix_gamma=True,
            name="batchnorm10_1_14_")
        relu10_1_14_ = mx.symbol.Activation(data=batchnorm10_1_14_,
            act_type='relu',
            name="relu10_1_14_")

        conv11_1_14_ = mx.symbol.Convolution(data=relu10_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_14_")
        # conv11_1_14_, output shape: {[256,56,56]}

        batchnorm11_1_14_ = mx.symbol.BatchNorm(data=conv11_1_14_,
            fix_gamma=True,
            name="batchnorm11_1_14_")
        conv9_1_15_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_15_")
        # conv9_1_15_, output shape: {[4,56,56]}

        batchnorm9_1_15_ = mx.symbol.BatchNorm(data=conv9_1_15_,
            fix_gamma=True,
            name="batchnorm9_1_15_")
        relu9_1_15_ = mx.symbol.Activation(data=batchnorm9_1_15_,
            act_type='relu',
            name="relu9_1_15_")

        conv10_1_15_ = mx.symbol.pad(data=relu9_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_15_ = mx.symbol.Convolution(data=conv10_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_15_")
        # conv10_1_15_, output shape: {[4,56,56]}

        batchnorm10_1_15_ = mx.symbol.BatchNorm(data=conv10_1_15_,
            fix_gamma=True,
            name="batchnorm10_1_15_")
        relu10_1_15_ = mx.symbol.Activation(data=batchnorm10_1_15_,
            act_type='relu',
            name="relu10_1_15_")

        conv11_1_15_ = mx.symbol.Convolution(data=relu10_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_15_")
        # conv11_1_15_, output shape: {[256,56,56]}

        batchnorm11_1_15_ = mx.symbol.BatchNorm(data=conv11_1_15_,
            fix_gamma=True,
            name="batchnorm11_1_15_")
        conv9_1_16_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_16_")
        # conv9_1_16_, output shape: {[4,56,56]}

        batchnorm9_1_16_ = mx.symbol.BatchNorm(data=conv9_1_16_,
            fix_gamma=True,
            name="batchnorm9_1_16_")
        relu9_1_16_ = mx.symbol.Activation(data=batchnorm9_1_16_,
            act_type='relu',
            name="relu9_1_16_")

        conv10_1_16_ = mx.symbol.pad(data=relu9_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_16_ = mx.symbol.Convolution(data=conv10_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_16_")
        # conv10_1_16_, output shape: {[4,56,56]}

        batchnorm10_1_16_ = mx.symbol.BatchNorm(data=conv10_1_16_,
            fix_gamma=True,
            name="batchnorm10_1_16_")
        relu10_1_16_ = mx.symbol.Activation(data=batchnorm10_1_16_,
            act_type='relu',
            name="relu10_1_16_")

        conv11_1_16_ = mx.symbol.Convolution(data=relu10_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_16_")
        # conv11_1_16_, output shape: {[256,56,56]}

        batchnorm11_1_16_ = mx.symbol.BatchNorm(data=conv11_1_16_,
            fix_gamma=True,
            name="batchnorm11_1_16_")
        conv9_1_17_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_17_")
        # conv9_1_17_, output shape: {[4,56,56]}

        batchnorm9_1_17_ = mx.symbol.BatchNorm(data=conv9_1_17_,
            fix_gamma=True,
            name="batchnorm9_1_17_")
        relu9_1_17_ = mx.symbol.Activation(data=batchnorm9_1_17_,
            act_type='relu',
            name="relu9_1_17_")

        conv10_1_17_ = mx.symbol.pad(data=relu9_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_17_ = mx.symbol.Convolution(data=conv10_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_17_")
        # conv10_1_17_, output shape: {[4,56,56]}

        batchnorm10_1_17_ = mx.symbol.BatchNorm(data=conv10_1_17_,
            fix_gamma=True,
            name="batchnorm10_1_17_")
        relu10_1_17_ = mx.symbol.Activation(data=batchnorm10_1_17_,
            act_type='relu',
            name="relu10_1_17_")

        conv11_1_17_ = mx.symbol.Convolution(data=relu10_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_17_")
        # conv11_1_17_, output shape: {[256,56,56]}

        batchnorm11_1_17_ = mx.symbol.BatchNorm(data=conv11_1_17_,
            fix_gamma=True,
            name="batchnorm11_1_17_")
        conv9_1_18_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_18_")
        # conv9_1_18_, output shape: {[4,56,56]}

        batchnorm9_1_18_ = mx.symbol.BatchNorm(data=conv9_1_18_,
            fix_gamma=True,
            name="batchnorm9_1_18_")
        relu9_1_18_ = mx.symbol.Activation(data=batchnorm9_1_18_,
            act_type='relu',
            name="relu9_1_18_")

        conv10_1_18_ = mx.symbol.pad(data=relu9_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_18_ = mx.symbol.Convolution(data=conv10_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_18_")
        # conv10_1_18_, output shape: {[4,56,56]}

        batchnorm10_1_18_ = mx.symbol.BatchNorm(data=conv10_1_18_,
            fix_gamma=True,
            name="batchnorm10_1_18_")
        relu10_1_18_ = mx.symbol.Activation(data=batchnorm10_1_18_,
            act_type='relu',
            name="relu10_1_18_")

        conv11_1_18_ = mx.symbol.Convolution(data=relu10_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_18_")
        # conv11_1_18_, output shape: {[256,56,56]}

        batchnorm11_1_18_ = mx.symbol.BatchNorm(data=conv11_1_18_,
            fix_gamma=True,
            name="batchnorm11_1_18_")
        conv9_1_19_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_19_")
        # conv9_1_19_, output shape: {[4,56,56]}

        batchnorm9_1_19_ = mx.symbol.BatchNorm(data=conv9_1_19_,
            fix_gamma=True,
            name="batchnorm9_1_19_")
        relu9_1_19_ = mx.symbol.Activation(data=batchnorm9_1_19_,
            act_type='relu',
            name="relu9_1_19_")

        conv10_1_19_ = mx.symbol.pad(data=relu9_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_19_ = mx.symbol.Convolution(data=conv10_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_19_")
        # conv10_1_19_, output shape: {[4,56,56]}

        batchnorm10_1_19_ = mx.symbol.BatchNorm(data=conv10_1_19_,
            fix_gamma=True,
            name="batchnorm10_1_19_")
        relu10_1_19_ = mx.symbol.Activation(data=batchnorm10_1_19_,
            act_type='relu',
            name="relu10_1_19_")

        conv11_1_19_ = mx.symbol.Convolution(data=relu10_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_19_")
        # conv11_1_19_, output shape: {[256,56,56]}

        batchnorm11_1_19_ = mx.symbol.BatchNorm(data=conv11_1_19_,
            fix_gamma=True,
            name="batchnorm11_1_19_")
        conv9_1_20_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_20_")
        # conv9_1_20_, output shape: {[4,56,56]}

        batchnorm9_1_20_ = mx.symbol.BatchNorm(data=conv9_1_20_,
            fix_gamma=True,
            name="batchnorm9_1_20_")
        relu9_1_20_ = mx.symbol.Activation(data=batchnorm9_1_20_,
            act_type='relu',
            name="relu9_1_20_")

        conv10_1_20_ = mx.symbol.pad(data=relu9_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_20_ = mx.symbol.Convolution(data=conv10_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_20_")
        # conv10_1_20_, output shape: {[4,56,56]}

        batchnorm10_1_20_ = mx.symbol.BatchNorm(data=conv10_1_20_,
            fix_gamma=True,
            name="batchnorm10_1_20_")
        relu10_1_20_ = mx.symbol.Activation(data=batchnorm10_1_20_,
            act_type='relu',
            name="relu10_1_20_")

        conv11_1_20_ = mx.symbol.Convolution(data=relu10_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_20_")
        # conv11_1_20_, output shape: {[256,56,56]}

        batchnorm11_1_20_ = mx.symbol.BatchNorm(data=conv11_1_20_,
            fix_gamma=True,
            name="batchnorm11_1_20_")
        conv9_1_21_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_21_")
        # conv9_1_21_, output shape: {[4,56,56]}

        batchnorm9_1_21_ = mx.symbol.BatchNorm(data=conv9_1_21_,
            fix_gamma=True,
            name="batchnorm9_1_21_")
        relu9_1_21_ = mx.symbol.Activation(data=batchnorm9_1_21_,
            act_type='relu',
            name="relu9_1_21_")

        conv10_1_21_ = mx.symbol.pad(data=relu9_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_21_ = mx.symbol.Convolution(data=conv10_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_21_")
        # conv10_1_21_, output shape: {[4,56,56]}

        batchnorm10_1_21_ = mx.symbol.BatchNorm(data=conv10_1_21_,
            fix_gamma=True,
            name="batchnorm10_1_21_")
        relu10_1_21_ = mx.symbol.Activation(data=batchnorm10_1_21_,
            act_type='relu',
            name="relu10_1_21_")

        conv11_1_21_ = mx.symbol.Convolution(data=relu10_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_21_")
        # conv11_1_21_, output shape: {[256,56,56]}

        batchnorm11_1_21_ = mx.symbol.BatchNorm(data=conv11_1_21_,
            fix_gamma=True,
            name="batchnorm11_1_21_")
        conv9_1_22_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_22_")
        # conv9_1_22_, output shape: {[4,56,56]}

        batchnorm9_1_22_ = mx.symbol.BatchNorm(data=conv9_1_22_,
            fix_gamma=True,
            name="batchnorm9_1_22_")
        relu9_1_22_ = mx.symbol.Activation(data=batchnorm9_1_22_,
            act_type='relu',
            name="relu9_1_22_")

        conv10_1_22_ = mx.symbol.pad(data=relu9_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_22_ = mx.symbol.Convolution(data=conv10_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_22_")
        # conv10_1_22_, output shape: {[4,56,56]}

        batchnorm10_1_22_ = mx.symbol.BatchNorm(data=conv10_1_22_,
            fix_gamma=True,
            name="batchnorm10_1_22_")
        relu10_1_22_ = mx.symbol.Activation(data=batchnorm10_1_22_,
            act_type='relu',
            name="relu10_1_22_")

        conv11_1_22_ = mx.symbol.Convolution(data=relu10_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_22_")
        # conv11_1_22_, output shape: {[256,56,56]}

        batchnorm11_1_22_ = mx.symbol.BatchNorm(data=conv11_1_22_,
            fix_gamma=True,
            name="batchnorm11_1_22_")
        conv9_1_23_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_23_")
        # conv9_1_23_, output shape: {[4,56,56]}

        batchnorm9_1_23_ = mx.symbol.BatchNorm(data=conv9_1_23_,
            fix_gamma=True,
            name="batchnorm9_1_23_")
        relu9_1_23_ = mx.symbol.Activation(data=batchnorm9_1_23_,
            act_type='relu',
            name="relu9_1_23_")

        conv10_1_23_ = mx.symbol.pad(data=relu9_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_23_ = mx.symbol.Convolution(data=conv10_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_23_")
        # conv10_1_23_, output shape: {[4,56,56]}

        batchnorm10_1_23_ = mx.symbol.BatchNorm(data=conv10_1_23_,
            fix_gamma=True,
            name="batchnorm10_1_23_")
        relu10_1_23_ = mx.symbol.Activation(data=batchnorm10_1_23_,
            act_type='relu',
            name="relu10_1_23_")

        conv11_1_23_ = mx.symbol.Convolution(data=relu10_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_23_")
        # conv11_1_23_, output shape: {[256,56,56]}

        batchnorm11_1_23_ = mx.symbol.BatchNorm(data=conv11_1_23_,
            fix_gamma=True,
            name="batchnorm11_1_23_")
        conv9_1_24_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_24_")
        # conv9_1_24_, output shape: {[4,56,56]}

        batchnorm9_1_24_ = mx.symbol.BatchNorm(data=conv9_1_24_,
            fix_gamma=True,
            name="batchnorm9_1_24_")
        relu9_1_24_ = mx.symbol.Activation(data=batchnorm9_1_24_,
            act_type='relu',
            name="relu9_1_24_")

        conv10_1_24_ = mx.symbol.pad(data=relu9_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_24_ = mx.symbol.Convolution(data=conv10_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_24_")
        # conv10_1_24_, output shape: {[4,56,56]}

        batchnorm10_1_24_ = mx.symbol.BatchNorm(data=conv10_1_24_,
            fix_gamma=True,
            name="batchnorm10_1_24_")
        relu10_1_24_ = mx.symbol.Activation(data=batchnorm10_1_24_,
            act_type='relu',
            name="relu10_1_24_")

        conv11_1_24_ = mx.symbol.Convolution(data=relu10_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_24_")
        # conv11_1_24_, output shape: {[256,56,56]}

        batchnorm11_1_24_ = mx.symbol.BatchNorm(data=conv11_1_24_,
            fix_gamma=True,
            name="batchnorm11_1_24_")
        conv9_1_25_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_25_")
        # conv9_1_25_, output shape: {[4,56,56]}

        batchnorm9_1_25_ = mx.symbol.BatchNorm(data=conv9_1_25_,
            fix_gamma=True,
            name="batchnorm9_1_25_")
        relu9_1_25_ = mx.symbol.Activation(data=batchnorm9_1_25_,
            act_type='relu',
            name="relu9_1_25_")

        conv10_1_25_ = mx.symbol.pad(data=relu9_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_25_ = mx.symbol.Convolution(data=conv10_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_25_")
        # conv10_1_25_, output shape: {[4,56,56]}

        batchnorm10_1_25_ = mx.symbol.BatchNorm(data=conv10_1_25_,
            fix_gamma=True,
            name="batchnorm10_1_25_")
        relu10_1_25_ = mx.symbol.Activation(data=batchnorm10_1_25_,
            act_type='relu',
            name="relu10_1_25_")

        conv11_1_25_ = mx.symbol.Convolution(data=relu10_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_25_")
        # conv11_1_25_, output shape: {[256,56,56]}

        batchnorm11_1_25_ = mx.symbol.BatchNorm(data=conv11_1_25_,
            fix_gamma=True,
            name="batchnorm11_1_25_")
        conv9_1_26_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_26_")
        # conv9_1_26_, output shape: {[4,56,56]}

        batchnorm9_1_26_ = mx.symbol.BatchNorm(data=conv9_1_26_,
            fix_gamma=True,
            name="batchnorm9_1_26_")
        relu9_1_26_ = mx.symbol.Activation(data=batchnorm9_1_26_,
            act_type='relu',
            name="relu9_1_26_")

        conv10_1_26_ = mx.symbol.pad(data=relu9_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_26_ = mx.symbol.Convolution(data=conv10_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_26_")
        # conv10_1_26_, output shape: {[4,56,56]}

        batchnorm10_1_26_ = mx.symbol.BatchNorm(data=conv10_1_26_,
            fix_gamma=True,
            name="batchnorm10_1_26_")
        relu10_1_26_ = mx.symbol.Activation(data=batchnorm10_1_26_,
            act_type='relu',
            name="relu10_1_26_")

        conv11_1_26_ = mx.symbol.Convolution(data=relu10_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_26_")
        # conv11_1_26_, output shape: {[256,56,56]}

        batchnorm11_1_26_ = mx.symbol.BatchNorm(data=conv11_1_26_,
            fix_gamma=True,
            name="batchnorm11_1_26_")
        conv9_1_27_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_27_")
        # conv9_1_27_, output shape: {[4,56,56]}

        batchnorm9_1_27_ = mx.symbol.BatchNorm(data=conv9_1_27_,
            fix_gamma=True,
            name="batchnorm9_1_27_")
        relu9_1_27_ = mx.symbol.Activation(data=batchnorm9_1_27_,
            act_type='relu',
            name="relu9_1_27_")

        conv10_1_27_ = mx.symbol.pad(data=relu9_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_27_ = mx.symbol.Convolution(data=conv10_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_27_")
        # conv10_1_27_, output shape: {[4,56,56]}

        batchnorm10_1_27_ = mx.symbol.BatchNorm(data=conv10_1_27_,
            fix_gamma=True,
            name="batchnorm10_1_27_")
        relu10_1_27_ = mx.symbol.Activation(data=batchnorm10_1_27_,
            act_type='relu',
            name="relu10_1_27_")

        conv11_1_27_ = mx.symbol.Convolution(data=relu10_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_27_")
        # conv11_1_27_, output shape: {[256,56,56]}

        batchnorm11_1_27_ = mx.symbol.BatchNorm(data=conv11_1_27_,
            fix_gamma=True,
            name="batchnorm11_1_27_")
        conv9_1_28_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_28_")
        # conv9_1_28_, output shape: {[4,56,56]}

        batchnorm9_1_28_ = mx.symbol.BatchNorm(data=conv9_1_28_,
            fix_gamma=True,
            name="batchnorm9_1_28_")
        relu9_1_28_ = mx.symbol.Activation(data=batchnorm9_1_28_,
            act_type='relu',
            name="relu9_1_28_")

        conv10_1_28_ = mx.symbol.pad(data=relu9_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_28_ = mx.symbol.Convolution(data=conv10_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_28_")
        # conv10_1_28_, output shape: {[4,56,56]}

        batchnorm10_1_28_ = mx.symbol.BatchNorm(data=conv10_1_28_,
            fix_gamma=True,
            name="batchnorm10_1_28_")
        relu10_1_28_ = mx.symbol.Activation(data=batchnorm10_1_28_,
            act_type='relu',
            name="relu10_1_28_")

        conv11_1_28_ = mx.symbol.Convolution(data=relu10_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_28_")
        # conv11_1_28_, output shape: {[256,56,56]}

        batchnorm11_1_28_ = mx.symbol.BatchNorm(data=conv11_1_28_,
            fix_gamma=True,
            name="batchnorm11_1_28_")
        conv9_1_29_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_29_")
        # conv9_1_29_, output shape: {[4,56,56]}

        batchnorm9_1_29_ = mx.symbol.BatchNorm(data=conv9_1_29_,
            fix_gamma=True,
            name="batchnorm9_1_29_")
        relu9_1_29_ = mx.symbol.Activation(data=batchnorm9_1_29_,
            act_type='relu',
            name="relu9_1_29_")

        conv10_1_29_ = mx.symbol.pad(data=relu9_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_29_ = mx.symbol.Convolution(data=conv10_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_29_")
        # conv10_1_29_, output shape: {[4,56,56]}

        batchnorm10_1_29_ = mx.symbol.BatchNorm(data=conv10_1_29_,
            fix_gamma=True,
            name="batchnorm10_1_29_")
        relu10_1_29_ = mx.symbol.Activation(data=batchnorm10_1_29_,
            act_type='relu',
            name="relu10_1_29_")

        conv11_1_29_ = mx.symbol.Convolution(data=relu10_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_29_")
        # conv11_1_29_, output shape: {[256,56,56]}

        batchnorm11_1_29_ = mx.symbol.BatchNorm(data=conv11_1_29_,
            fix_gamma=True,
            name="batchnorm11_1_29_")
        conv9_1_30_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_30_")
        # conv9_1_30_, output shape: {[4,56,56]}

        batchnorm9_1_30_ = mx.symbol.BatchNorm(data=conv9_1_30_,
            fix_gamma=True,
            name="batchnorm9_1_30_")
        relu9_1_30_ = mx.symbol.Activation(data=batchnorm9_1_30_,
            act_type='relu',
            name="relu9_1_30_")

        conv10_1_30_ = mx.symbol.pad(data=relu9_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_30_ = mx.symbol.Convolution(data=conv10_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_30_")
        # conv10_1_30_, output shape: {[4,56,56]}

        batchnorm10_1_30_ = mx.symbol.BatchNorm(data=conv10_1_30_,
            fix_gamma=True,
            name="batchnorm10_1_30_")
        relu10_1_30_ = mx.symbol.Activation(data=batchnorm10_1_30_,
            act_type='relu',
            name="relu10_1_30_")

        conv11_1_30_ = mx.symbol.Convolution(data=relu10_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_30_")
        # conv11_1_30_, output shape: {[256,56,56]}

        batchnorm11_1_30_ = mx.symbol.BatchNorm(data=conv11_1_30_,
            fix_gamma=True,
            name="batchnorm11_1_30_")
        conv9_1_31_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_31_")
        # conv9_1_31_, output shape: {[4,56,56]}

        batchnorm9_1_31_ = mx.symbol.BatchNorm(data=conv9_1_31_,
            fix_gamma=True,
            name="batchnorm9_1_31_")
        relu9_1_31_ = mx.symbol.Activation(data=batchnorm9_1_31_,
            act_type='relu',
            name="relu9_1_31_")

        conv10_1_31_ = mx.symbol.pad(data=relu9_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_31_ = mx.symbol.Convolution(data=conv10_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_31_")
        # conv10_1_31_, output shape: {[4,56,56]}

        batchnorm10_1_31_ = mx.symbol.BatchNorm(data=conv10_1_31_,
            fix_gamma=True,
            name="batchnorm10_1_31_")
        relu10_1_31_ = mx.symbol.Activation(data=batchnorm10_1_31_,
            act_type='relu',
            name="relu10_1_31_")

        conv11_1_31_ = mx.symbol.Convolution(data=relu10_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_31_")
        # conv11_1_31_, output shape: {[256,56,56]}

        batchnorm11_1_31_ = mx.symbol.BatchNorm(data=conv11_1_31_,
            fix_gamma=True,
            name="batchnorm11_1_31_")
        conv9_1_32_ = mx.symbol.Convolution(data=relu7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv9_1_32_")
        # conv9_1_32_, output shape: {[4,56,56]}

        batchnorm9_1_32_ = mx.symbol.BatchNorm(data=conv9_1_32_,
            fix_gamma=True,
            name="batchnorm9_1_32_")
        relu9_1_32_ = mx.symbol.Activation(data=batchnorm9_1_32_,
            act_type='relu',
            name="relu9_1_32_")

        conv10_1_32_ = mx.symbol.pad(data=relu9_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_1_32_ = mx.symbol.Convolution(data=conv10_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv10_1_32_")
        # conv10_1_32_, output shape: {[4,56,56]}

        batchnorm10_1_32_ = mx.symbol.BatchNorm(data=conv10_1_32_,
            fix_gamma=True,
            name="batchnorm10_1_32_")
        relu10_1_32_ = mx.symbol.Activation(data=batchnorm10_1_32_,
            act_type='relu',
            name="relu10_1_32_")

        conv11_1_32_ = mx.symbol.Convolution(data=relu10_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv11_1_32_")
        # conv11_1_32_, output shape: {[256,56,56]}

        batchnorm11_1_32_ = mx.symbol.BatchNorm(data=conv11_1_32_,
            fix_gamma=True,
            name="batchnorm11_1_32_")
        add12_1_ = batchnorm11_1_1_ + batchnorm11_1_2_ + batchnorm11_1_3_ + batchnorm11_1_4_ + batchnorm11_1_5_ + batchnorm11_1_6_ + batchnorm11_1_7_ + batchnorm11_1_8_ + batchnorm11_1_9_ + batchnorm11_1_10_ + batchnorm11_1_11_ + batchnorm11_1_12_ + batchnorm11_1_13_ + batchnorm11_1_14_ + batchnorm11_1_15_ + batchnorm11_1_16_ + batchnorm11_1_17_ + batchnorm11_1_18_ + batchnorm11_1_19_ + batchnorm11_1_20_ + batchnorm11_1_21_ + batchnorm11_1_22_ + batchnorm11_1_23_ + batchnorm11_1_24_ + batchnorm11_1_25_ + batchnorm11_1_26_ + batchnorm11_1_27_ + batchnorm11_1_28_ + batchnorm11_1_29_ + batchnorm11_1_30_ + batchnorm11_1_31_ + batchnorm11_1_32_
        # add12_1_, output shape: {[256,56,56]}

        add13_ = add12_1_ + relu7_
        # add13_, output shape: {[256,56,56]}

        relu13_ = mx.symbol.Activation(data=add13_,
            act_type='relu',
            name="relu13_")

        conv15_1_1_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_1_")
        # conv15_1_1_, output shape: {[4,56,56]}

        batchnorm15_1_1_ = mx.symbol.BatchNorm(data=conv15_1_1_,
            fix_gamma=True,
            name="batchnorm15_1_1_")
        relu15_1_1_ = mx.symbol.Activation(data=batchnorm15_1_1_,
            act_type='relu',
            name="relu15_1_1_")

        conv16_1_1_ = mx.symbol.pad(data=relu15_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_1_ = mx.symbol.Convolution(data=conv16_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_1_")
        # conv16_1_1_, output shape: {[4,56,56]}

        batchnorm16_1_1_ = mx.symbol.BatchNorm(data=conv16_1_1_,
            fix_gamma=True,
            name="batchnorm16_1_1_")
        relu16_1_1_ = mx.symbol.Activation(data=batchnorm16_1_1_,
            act_type='relu',
            name="relu16_1_1_")

        conv17_1_1_ = mx.symbol.Convolution(data=relu16_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_1_")
        # conv17_1_1_, output shape: {[256,56,56]}

        batchnorm17_1_1_ = mx.symbol.BatchNorm(data=conv17_1_1_,
            fix_gamma=True,
            name="batchnorm17_1_1_")
        conv15_1_2_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_2_")
        # conv15_1_2_, output shape: {[4,56,56]}

        batchnorm15_1_2_ = mx.symbol.BatchNorm(data=conv15_1_2_,
            fix_gamma=True,
            name="batchnorm15_1_2_")
        relu15_1_2_ = mx.symbol.Activation(data=batchnorm15_1_2_,
            act_type='relu',
            name="relu15_1_2_")

        conv16_1_2_ = mx.symbol.pad(data=relu15_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_2_ = mx.symbol.Convolution(data=conv16_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_2_")
        # conv16_1_2_, output shape: {[4,56,56]}

        batchnorm16_1_2_ = mx.symbol.BatchNorm(data=conv16_1_2_,
            fix_gamma=True,
            name="batchnorm16_1_2_")
        relu16_1_2_ = mx.symbol.Activation(data=batchnorm16_1_2_,
            act_type='relu',
            name="relu16_1_2_")

        conv17_1_2_ = mx.symbol.Convolution(data=relu16_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_2_")
        # conv17_1_2_, output shape: {[256,56,56]}

        batchnorm17_1_2_ = mx.symbol.BatchNorm(data=conv17_1_2_,
            fix_gamma=True,
            name="batchnorm17_1_2_")
        conv15_1_3_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_3_")
        # conv15_1_3_, output shape: {[4,56,56]}

        batchnorm15_1_3_ = mx.symbol.BatchNorm(data=conv15_1_3_,
            fix_gamma=True,
            name="batchnorm15_1_3_")
        relu15_1_3_ = mx.symbol.Activation(data=batchnorm15_1_3_,
            act_type='relu',
            name="relu15_1_3_")

        conv16_1_3_ = mx.symbol.pad(data=relu15_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_3_ = mx.symbol.Convolution(data=conv16_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_3_")
        # conv16_1_3_, output shape: {[4,56,56]}

        batchnorm16_1_3_ = mx.symbol.BatchNorm(data=conv16_1_3_,
            fix_gamma=True,
            name="batchnorm16_1_3_")
        relu16_1_3_ = mx.symbol.Activation(data=batchnorm16_1_3_,
            act_type='relu',
            name="relu16_1_3_")

        conv17_1_3_ = mx.symbol.Convolution(data=relu16_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_3_")
        # conv17_1_3_, output shape: {[256,56,56]}

        batchnorm17_1_3_ = mx.symbol.BatchNorm(data=conv17_1_3_,
            fix_gamma=True,
            name="batchnorm17_1_3_")
        conv15_1_4_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_4_")
        # conv15_1_4_, output shape: {[4,56,56]}

        batchnorm15_1_4_ = mx.symbol.BatchNorm(data=conv15_1_4_,
            fix_gamma=True,
            name="batchnorm15_1_4_")
        relu15_1_4_ = mx.symbol.Activation(data=batchnorm15_1_4_,
            act_type='relu',
            name="relu15_1_4_")

        conv16_1_4_ = mx.symbol.pad(data=relu15_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_4_ = mx.symbol.Convolution(data=conv16_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_4_")
        # conv16_1_4_, output shape: {[4,56,56]}

        batchnorm16_1_4_ = mx.symbol.BatchNorm(data=conv16_1_4_,
            fix_gamma=True,
            name="batchnorm16_1_4_")
        relu16_1_4_ = mx.symbol.Activation(data=batchnorm16_1_4_,
            act_type='relu',
            name="relu16_1_4_")

        conv17_1_4_ = mx.symbol.Convolution(data=relu16_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_4_")
        # conv17_1_4_, output shape: {[256,56,56]}

        batchnorm17_1_4_ = mx.symbol.BatchNorm(data=conv17_1_4_,
            fix_gamma=True,
            name="batchnorm17_1_4_")
        conv15_1_5_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_5_")
        # conv15_1_5_, output shape: {[4,56,56]}

        batchnorm15_1_5_ = mx.symbol.BatchNorm(data=conv15_1_5_,
            fix_gamma=True,
            name="batchnorm15_1_5_")
        relu15_1_5_ = mx.symbol.Activation(data=batchnorm15_1_5_,
            act_type='relu',
            name="relu15_1_5_")

        conv16_1_5_ = mx.symbol.pad(data=relu15_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_5_ = mx.symbol.Convolution(data=conv16_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_5_")
        # conv16_1_5_, output shape: {[4,56,56]}

        batchnorm16_1_5_ = mx.symbol.BatchNorm(data=conv16_1_5_,
            fix_gamma=True,
            name="batchnorm16_1_5_")
        relu16_1_5_ = mx.symbol.Activation(data=batchnorm16_1_5_,
            act_type='relu',
            name="relu16_1_5_")

        conv17_1_5_ = mx.symbol.Convolution(data=relu16_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_5_")
        # conv17_1_5_, output shape: {[256,56,56]}

        batchnorm17_1_5_ = mx.symbol.BatchNorm(data=conv17_1_5_,
            fix_gamma=True,
            name="batchnorm17_1_5_")
        conv15_1_6_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_6_")
        # conv15_1_6_, output shape: {[4,56,56]}

        batchnorm15_1_6_ = mx.symbol.BatchNorm(data=conv15_1_6_,
            fix_gamma=True,
            name="batchnorm15_1_6_")
        relu15_1_6_ = mx.symbol.Activation(data=batchnorm15_1_6_,
            act_type='relu',
            name="relu15_1_6_")

        conv16_1_6_ = mx.symbol.pad(data=relu15_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_6_ = mx.symbol.Convolution(data=conv16_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_6_")
        # conv16_1_6_, output shape: {[4,56,56]}

        batchnorm16_1_6_ = mx.symbol.BatchNorm(data=conv16_1_6_,
            fix_gamma=True,
            name="batchnorm16_1_6_")
        relu16_1_6_ = mx.symbol.Activation(data=batchnorm16_1_6_,
            act_type='relu',
            name="relu16_1_6_")

        conv17_1_6_ = mx.symbol.Convolution(data=relu16_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_6_")
        # conv17_1_6_, output shape: {[256,56,56]}

        batchnorm17_1_6_ = mx.symbol.BatchNorm(data=conv17_1_6_,
            fix_gamma=True,
            name="batchnorm17_1_6_")
        conv15_1_7_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_7_")
        # conv15_1_7_, output shape: {[4,56,56]}

        batchnorm15_1_7_ = mx.symbol.BatchNorm(data=conv15_1_7_,
            fix_gamma=True,
            name="batchnorm15_1_7_")
        relu15_1_7_ = mx.symbol.Activation(data=batchnorm15_1_7_,
            act_type='relu',
            name="relu15_1_7_")

        conv16_1_7_ = mx.symbol.pad(data=relu15_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_7_ = mx.symbol.Convolution(data=conv16_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_7_")
        # conv16_1_7_, output shape: {[4,56,56]}

        batchnorm16_1_7_ = mx.symbol.BatchNorm(data=conv16_1_7_,
            fix_gamma=True,
            name="batchnorm16_1_7_")
        relu16_1_7_ = mx.symbol.Activation(data=batchnorm16_1_7_,
            act_type='relu',
            name="relu16_1_7_")

        conv17_1_7_ = mx.symbol.Convolution(data=relu16_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_7_")
        # conv17_1_7_, output shape: {[256,56,56]}

        batchnorm17_1_7_ = mx.symbol.BatchNorm(data=conv17_1_7_,
            fix_gamma=True,
            name="batchnorm17_1_7_")
        conv15_1_8_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_8_")
        # conv15_1_8_, output shape: {[4,56,56]}

        batchnorm15_1_8_ = mx.symbol.BatchNorm(data=conv15_1_8_,
            fix_gamma=True,
            name="batchnorm15_1_8_")
        relu15_1_8_ = mx.symbol.Activation(data=batchnorm15_1_8_,
            act_type='relu',
            name="relu15_1_8_")

        conv16_1_8_ = mx.symbol.pad(data=relu15_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_8_ = mx.symbol.Convolution(data=conv16_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_8_")
        # conv16_1_8_, output shape: {[4,56,56]}

        batchnorm16_1_8_ = mx.symbol.BatchNorm(data=conv16_1_8_,
            fix_gamma=True,
            name="batchnorm16_1_8_")
        relu16_1_8_ = mx.symbol.Activation(data=batchnorm16_1_8_,
            act_type='relu',
            name="relu16_1_8_")

        conv17_1_8_ = mx.symbol.Convolution(data=relu16_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_8_")
        # conv17_1_8_, output shape: {[256,56,56]}

        batchnorm17_1_8_ = mx.symbol.BatchNorm(data=conv17_1_8_,
            fix_gamma=True,
            name="batchnorm17_1_8_")
        conv15_1_9_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_9_")
        # conv15_1_9_, output shape: {[4,56,56]}

        batchnorm15_1_9_ = mx.symbol.BatchNorm(data=conv15_1_9_,
            fix_gamma=True,
            name="batchnorm15_1_9_")
        relu15_1_9_ = mx.symbol.Activation(data=batchnorm15_1_9_,
            act_type='relu',
            name="relu15_1_9_")

        conv16_1_9_ = mx.symbol.pad(data=relu15_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_9_ = mx.symbol.Convolution(data=conv16_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_9_")
        # conv16_1_9_, output shape: {[4,56,56]}

        batchnorm16_1_9_ = mx.symbol.BatchNorm(data=conv16_1_9_,
            fix_gamma=True,
            name="batchnorm16_1_9_")
        relu16_1_9_ = mx.symbol.Activation(data=batchnorm16_1_9_,
            act_type='relu',
            name="relu16_1_9_")

        conv17_1_9_ = mx.symbol.Convolution(data=relu16_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_9_")
        # conv17_1_9_, output shape: {[256,56,56]}

        batchnorm17_1_9_ = mx.symbol.BatchNorm(data=conv17_1_9_,
            fix_gamma=True,
            name="batchnorm17_1_9_")
        conv15_1_10_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_10_")
        # conv15_1_10_, output shape: {[4,56,56]}

        batchnorm15_1_10_ = mx.symbol.BatchNorm(data=conv15_1_10_,
            fix_gamma=True,
            name="batchnorm15_1_10_")
        relu15_1_10_ = mx.symbol.Activation(data=batchnorm15_1_10_,
            act_type='relu',
            name="relu15_1_10_")

        conv16_1_10_ = mx.symbol.pad(data=relu15_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_10_ = mx.symbol.Convolution(data=conv16_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_10_")
        # conv16_1_10_, output shape: {[4,56,56]}

        batchnorm16_1_10_ = mx.symbol.BatchNorm(data=conv16_1_10_,
            fix_gamma=True,
            name="batchnorm16_1_10_")
        relu16_1_10_ = mx.symbol.Activation(data=batchnorm16_1_10_,
            act_type='relu',
            name="relu16_1_10_")

        conv17_1_10_ = mx.symbol.Convolution(data=relu16_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_10_")
        # conv17_1_10_, output shape: {[256,56,56]}

        batchnorm17_1_10_ = mx.symbol.BatchNorm(data=conv17_1_10_,
            fix_gamma=True,
            name="batchnorm17_1_10_")
        conv15_1_11_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_11_")
        # conv15_1_11_, output shape: {[4,56,56]}

        batchnorm15_1_11_ = mx.symbol.BatchNorm(data=conv15_1_11_,
            fix_gamma=True,
            name="batchnorm15_1_11_")
        relu15_1_11_ = mx.symbol.Activation(data=batchnorm15_1_11_,
            act_type='relu',
            name="relu15_1_11_")

        conv16_1_11_ = mx.symbol.pad(data=relu15_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_11_ = mx.symbol.Convolution(data=conv16_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_11_")
        # conv16_1_11_, output shape: {[4,56,56]}

        batchnorm16_1_11_ = mx.symbol.BatchNorm(data=conv16_1_11_,
            fix_gamma=True,
            name="batchnorm16_1_11_")
        relu16_1_11_ = mx.symbol.Activation(data=batchnorm16_1_11_,
            act_type='relu',
            name="relu16_1_11_")

        conv17_1_11_ = mx.symbol.Convolution(data=relu16_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_11_")
        # conv17_1_11_, output shape: {[256,56,56]}

        batchnorm17_1_11_ = mx.symbol.BatchNorm(data=conv17_1_11_,
            fix_gamma=True,
            name="batchnorm17_1_11_")
        conv15_1_12_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_12_")
        # conv15_1_12_, output shape: {[4,56,56]}

        batchnorm15_1_12_ = mx.symbol.BatchNorm(data=conv15_1_12_,
            fix_gamma=True,
            name="batchnorm15_1_12_")
        relu15_1_12_ = mx.symbol.Activation(data=batchnorm15_1_12_,
            act_type='relu',
            name="relu15_1_12_")

        conv16_1_12_ = mx.symbol.pad(data=relu15_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_12_ = mx.symbol.Convolution(data=conv16_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_12_")
        # conv16_1_12_, output shape: {[4,56,56]}

        batchnorm16_1_12_ = mx.symbol.BatchNorm(data=conv16_1_12_,
            fix_gamma=True,
            name="batchnorm16_1_12_")
        relu16_1_12_ = mx.symbol.Activation(data=batchnorm16_1_12_,
            act_type='relu',
            name="relu16_1_12_")

        conv17_1_12_ = mx.symbol.Convolution(data=relu16_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_12_")
        # conv17_1_12_, output shape: {[256,56,56]}

        batchnorm17_1_12_ = mx.symbol.BatchNorm(data=conv17_1_12_,
            fix_gamma=True,
            name="batchnorm17_1_12_")
        conv15_1_13_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_13_")
        # conv15_1_13_, output shape: {[4,56,56]}

        batchnorm15_1_13_ = mx.symbol.BatchNorm(data=conv15_1_13_,
            fix_gamma=True,
            name="batchnorm15_1_13_")
        relu15_1_13_ = mx.symbol.Activation(data=batchnorm15_1_13_,
            act_type='relu',
            name="relu15_1_13_")

        conv16_1_13_ = mx.symbol.pad(data=relu15_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_13_ = mx.symbol.Convolution(data=conv16_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_13_")
        # conv16_1_13_, output shape: {[4,56,56]}

        batchnorm16_1_13_ = mx.symbol.BatchNorm(data=conv16_1_13_,
            fix_gamma=True,
            name="batchnorm16_1_13_")
        relu16_1_13_ = mx.symbol.Activation(data=batchnorm16_1_13_,
            act_type='relu',
            name="relu16_1_13_")

        conv17_1_13_ = mx.symbol.Convolution(data=relu16_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_13_")
        # conv17_1_13_, output shape: {[256,56,56]}

        batchnorm17_1_13_ = mx.symbol.BatchNorm(data=conv17_1_13_,
            fix_gamma=True,
            name="batchnorm17_1_13_")
        conv15_1_14_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_14_")
        # conv15_1_14_, output shape: {[4,56,56]}

        batchnorm15_1_14_ = mx.symbol.BatchNorm(data=conv15_1_14_,
            fix_gamma=True,
            name="batchnorm15_1_14_")
        relu15_1_14_ = mx.symbol.Activation(data=batchnorm15_1_14_,
            act_type='relu',
            name="relu15_1_14_")

        conv16_1_14_ = mx.symbol.pad(data=relu15_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_14_ = mx.symbol.Convolution(data=conv16_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_14_")
        # conv16_1_14_, output shape: {[4,56,56]}

        batchnorm16_1_14_ = mx.symbol.BatchNorm(data=conv16_1_14_,
            fix_gamma=True,
            name="batchnorm16_1_14_")
        relu16_1_14_ = mx.symbol.Activation(data=batchnorm16_1_14_,
            act_type='relu',
            name="relu16_1_14_")

        conv17_1_14_ = mx.symbol.Convolution(data=relu16_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_14_")
        # conv17_1_14_, output shape: {[256,56,56]}

        batchnorm17_1_14_ = mx.symbol.BatchNorm(data=conv17_1_14_,
            fix_gamma=True,
            name="batchnorm17_1_14_")
        conv15_1_15_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_15_")
        # conv15_1_15_, output shape: {[4,56,56]}

        batchnorm15_1_15_ = mx.symbol.BatchNorm(data=conv15_1_15_,
            fix_gamma=True,
            name="batchnorm15_1_15_")
        relu15_1_15_ = mx.symbol.Activation(data=batchnorm15_1_15_,
            act_type='relu',
            name="relu15_1_15_")

        conv16_1_15_ = mx.symbol.pad(data=relu15_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_15_ = mx.symbol.Convolution(data=conv16_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_15_")
        # conv16_1_15_, output shape: {[4,56,56]}

        batchnorm16_1_15_ = mx.symbol.BatchNorm(data=conv16_1_15_,
            fix_gamma=True,
            name="batchnorm16_1_15_")
        relu16_1_15_ = mx.symbol.Activation(data=batchnorm16_1_15_,
            act_type='relu',
            name="relu16_1_15_")

        conv17_1_15_ = mx.symbol.Convolution(data=relu16_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_15_")
        # conv17_1_15_, output shape: {[256,56,56]}

        batchnorm17_1_15_ = mx.symbol.BatchNorm(data=conv17_1_15_,
            fix_gamma=True,
            name="batchnorm17_1_15_")
        conv15_1_16_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_16_")
        # conv15_1_16_, output shape: {[4,56,56]}

        batchnorm15_1_16_ = mx.symbol.BatchNorm(data=conv15_1_16_,
            fix_gamma=True,
            name="batchnorm15_1_16_")
        relu15_1_16_ = mx.symbol.Activation(data=batchnorm15_1_16_,
            act_type='relu',
            name="relu15_1_16_")

        conv16_1_16_ = mx.symbol.pad(data=relu15_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_16_ = mx.symbol.Convolution(data=conv16_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_16_")
        # conv16_1_16_, output shape: {[4,56,56]}

        batchnorm16_1_16_ = mx.symbol.BatchNorm(data=conv16_1_16_,
            fix_gamma=True,
            name="batchnorm16_1_16_")
        relu16_1_16_ = mx.symbol.Activation(data=batchnorm16_1_16_,
            act_type='relu',
            name="relu16_1_16_")

        conv17_1_16_ = mx.symbol.Convolution(data=relu16_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_16_")
        # conv17_1_16_, output shape: {[256,56,56]}

        batchnorm17_1_16_ = mx.symbol.BatchNorm(data=conv17_1_16_,
            fix_gamma=True,
            name="batchnorm17_1_16_")
        conv15_1_17_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_17_")
        # conv15_1_17_, output shape: {[4,56,56]}

        batchnorm15_1_17_ = mx.symbol.BatchNorm(data=conv15_1_17_,
            fix_gamma=True,
            name="batchnorm15_1_17_")
        relu15_1_17_ = mx.symbol.Activation(data=batchnorm15_1_17_,
            act_type='relu',
            name="relu15_1_17_")

        conv16_1_17_ = mx.symbol.pad(data=relu15_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_17_ = mx.symbol.Convolution(data=conv16_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_17_")
        # conv16_1_17_, output shape: {[4,56,56]}

        batchnorm16_1_17_ = mx.symbol.BatchNorm(data=conv16_1_17_,
            fix_gamma=True,
            name="batchnorm16_1_17_")
        relu16_1_17_ = mx.symbol.Activation(data=batchnorm16_1_17_,
            act_type='relu',
            name="relu16_1_17_")

        conv17_1_17_ = mx.symbol.Convolution(data=relu16_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_17_")
        # conv17_1_17_, output shape: {[256,56,56]}

        batchnorm17_1_17_ = mx.symbol.BatchNorm(data=conv17_1_17_,
            fix_gamma=True,
            name="batchnorm17_1_17_")
        conv15_1_18_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_18_")
        # conv15_1_18_, output shape: {[4,56,56]}

        batchnorm15_1_18_ = mx.symbol.BatchNorm(data=conv15_1_18_,
            fix_gamma=True,
            name="batchnorm15_1_18_")
        relu15_1_18_ = mx.symbol.Activation(data=batchnorm15_1_18_,
            act_type='relu',
            name="relu15_1_18_")

        conv16_1_18_ = mx.symbol.pad(data=relu15_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_18_ = mx.symbol.Convolution(data=conv16_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_18_")
        # conv16_1_18_, output shape: {[4,56,56]}

        batchnorm16_1_18_ = mx.symbol.BatchNorm(data=conv16_1_18_,
            fix_gamma=True,
            name="batchnorm16_1_18_")
        relu16_1_18_ = mx.symbol.Activation(data=batchnorm16_1_18_,
            act_type='relu',
            name="relu16_1_18_")

        conv17_1_18_ = mx.symbol.Convolution(data=relu16_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_18_")
        # conv17_1_18_, output shape: {[256,56,56]}

        batchnorm17_1_18_ = mx.symbol.BatchNorm(data=conv17_1_18_,
            fix_gamma=True,
            name="batchnorm17_1_18_")
        conv15_1_19_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_19_")
        # conv15_1_19_, output shape: {[4,56,56]}

        batchnorm15_1_19_ = mx.symbol.BatchNorm(data=conv15_1_19_,
            fix_gamma=True,
            name="batchnorm15_1_19_")
        relu15_1_19_ = mx.symbol.Activation(data=batchnorm15_1_19_,
            act_type='relu',
            name="relu15_1_19_")

        conv16_1_19_ = mx.symbol.pad(data=relu15_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_19_ = mx.symbol.Convolution(data=conv16_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_19_")
        # conv16_1_19_, output shape: {[4,56,56]}

        batchnorm16_1_19_ = mx.symbol.BatchNorm(data=conv16_1_19_,
            fix_gamma=True,
            name="batchnorm16_1_19_")
        relu16_1_19_ = mx.symbol.Activation(data=batchnorm16_1_19_,
            act_type='relu',
            name="relu16_1_19_")

        conv17_1_19_ = mx.symbol.Convolution(data=relu16_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_19_")
        # conv17_1_19_, output shape: {[256,56,56]}

        batchnorm17_1_19_ = mx.symbol.BatchNorm(data=conv17_1_19_,
            fix_gamma=True,
            name="batchnorm17_1_19_")
        conv15_1_20_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_20_")
        # conv15_1_20_, output shape: {[4,56,56]}

        batchnorm15_1_20_ = mx.symbol.BatchNorm(data=conv15_1_20_,
            fix_gamma=True,
            name="batchnorm15_1_20_")
        relu15_1_20_ = mx.symbol.Activation(data=batchnorm15_1_20_,
            act_type='relu',
            name="relu15_1_20_")

        conv16_1_20_ = mx.symbol.pad(data=relu15_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_20_ = mx.symbol.Convolution(data=conv16_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_20_")
        # conv16_1_20_, output shape: {[4,56,56]}

        batchnorm16_1_20_ = mx.symbol.BatchNorm(data=conv16_1_20_,
            fix_gamma=True,
            name="batchnorm16_1_20_")
        relu16_1_20_ = mx.symbol.Activation(data=batchnorm16_1_20_,
            act_type='relu',
            name="relu16_1_20_")

        conv17_1_20_ = mx.symbol.Convolution(data=relu16_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_20_")
        # conv17_1_20_, output shape: {[256,56,56]}

        batchnorm17_1_20_ = mx.symbol.BatchNorm(data=conv17_1_20_,
            fix_gamma=True,
            name="batchnorm17_1_20_")
        conv15_1_21_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_21_")
        # conv15_1_21_, output shape: {[4,56,56]}

        batchnorm15_1_21_ = mx.symbol.BatchNorm(data=conv15_1_21_,
            fix_gamma=True,
            name="batchnorm15_1_21_")
        relu15_1_21_ = mx.symbol.Activation(data=batchnorm15_1_21_,
            act_type='relu',
            name="relu15_1_21_")

        conv16_1_21_ = mx.symbol.pad(data=relu15_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_21_ = mx.symbol.Convolution(data=conv16_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_21_")
        # conv16_1_21_, output shape: {[4,56,56]}

        batchnorm16_1_21_ = mx.symbol.BatchNorm(data=conv16_1_21_,
            fix_gamma=True,
            name="batchnorm16_1_21_")
        relu16_1_21_ = mx.symbol.Activation(data=batchnorm16_1_21_,
            act_type='relu',
            name="relu16_1_21_")

        conv17_1_21_ = mx.symbol.Convolution(data=relu16_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_21_")
        # conv17_1_21_, output shape: {[256,56,56]}

        batchnorm17_1_21_ = mx.symbol.BatchNorm(data=conv17_1_21_,
            fix_gamma=True,
            name="batchnorm17_1_21_")
        conv15_1_22_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_22_")
        # conv15_1_22_, output shape: {[4,56,56]}

        batchnorm15_1_22_ = mx.symbol.BatchNorm(data=conv15_1_22_,
            fix_gamma=True,
            name="batchnorm15_1_22_")
        relu15_1_22_ = mx.symbol.Activation(data=batchnorm15_1_22_,
            act_type='relu',
            name="relu15_1_22_")

        conv16_1_22_ = mx.symbol.pad(data=relu15_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_22_ = mx.symbol.Convolution(data=conv16_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_22_")
        # conv16_1_22_, output shape: {[4,56,56]}

        batchnorm16_1_22_ = mx.symbol.BatchNorm(data=conv16_1_22_,
            fix_gamma=True,
            name="batchnorm16_1_22_")
        relu16_1_22_ = mx.symbol.Activation(data=batchnorm16_1_22_,
            act_type='relu',
            name="relu16_1_22_")

        conv17_1_22_ = mx.symbol.Convolution(data=relu16_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_22_")
        # conv17_1_22_, output shape: {[256,56,56]}

        batchnorm17_1_22_ = mx.symbol.BatchNorm(data=conv17_1_22_,
            fix_gamma=True,
            name="batchnorm17_1_22_")
        conv15_1_23_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_23_")
        # conv15_1_23_, output shape: {[4,56,56]}

        batchnorm15_1_23_ = mx.symbol.BatchNorm(data=conv15_1_23_,
            fix_gamma=True,
            name="batchnorm15_1_23_")
        relu15_1_23_ = mx.symbol.Activation(data=batchnorm15_1_23_,
            act_type='relu',
            name="relu15_1_23_")

        conv16_1_23_ = mx.symbol.pad(data=relu15_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_23_ = mx.symbol.Convolution(data=conv16_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_23_")
        # conv16_1_23_, output shape: {[4,56,56]}

        batchnorm16_1_23_ = mx.symbol.BatchNorm(data=conv16_1_23_,
            fix_gamma=True,
            name="batchnorm16_1_23_")
        relu16_1_23_ = mx.symbol.Activation(data=batchnorm16_1_23_,
            act_type='relu',
            name="relu16_1_23_")

        conv17_1_23_ = mx.symbol.Convolution(data=relu16_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_23_")
        # conv17_1_23_, output shape: {[256,56,56]}

        batchnorm17_1_23_ = mx.symbol.BatchNorm(data=conv17_1_23_,
            fix_gamma=True,
            name="batchnorm17_1_23_")
        conv15_1_24_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_24_")
        # conv15_1_24_, output shape: {[4,56,56]}

        batchnorm15_1_24_ = mx.symbol.BatchNorm(data=conv15_1_24_,
            fix_gamma=True,
            name="batchnorm15_1_24_")
        relu15_1_24_ = mx.symbol.Activation(data=batchnorm15_1_24_,
            act_type='relu',
            name="relu15_1_24_")

        conv16_1_24_ = mx.symbol.pad(data=relu15_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_24_ = mx.symbol.Convolution(data=conv16_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_24_")
        # conv16_1_24_, output shape: {[4,56,56]}

        batchnorm16_1_24_ = mx.symbol.BatchNorm(data=conv16_1_24_,
            fix_gamma=True,
            name="batchnorm16_1_24_")
        relu16_1_24_ = mx.symbol.Activation(data=batchnorm16_1_24_,
            act_type='relu',
            name="relu16_1_24_")

        conv17_1_24_ = mx.symbol.Convolution(data=relu16_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_24_")
        # conv17_1_24_, output shape: {[256,56,56]}

        batchnorm17_1_24_ = mx.symbol.BatchNorm(data=conv17_1_24_,
            fix_gamma=True,
            name="batchnorm17_1_24_")
        conv15_1_25_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_25_")
        # conv15_1_25_, output shape: {[4,56,56]}

        batchnorm15_1_25_ = mx.symbol.BatchNorm(data=conv15_1_25_,
            fix_gamma=True,
            name="batchnorm15_1_25_")
        relu15_1_25_ = mx.symbol.Activation(data=batchnorm15_1_25_,
            act_type='relu',
            name="relu15_1_25_")

        conv16_1_25_ = mx.symbol.pad(data=relu15_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_25_ = mx.symbol.Convolution(data=conv16_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_25_")
        # conv16_1_25_, output shape: {[4,56,56]}

        batchnorm16_1_25_ = mx.symbol.BatchNorm(data=conv16_1_25_,
            fix_gamma=True,
            name="batchnorm16_1_25_")
        relu16_1_25_ = mx.symbol.Activation(data=batchnorm16_1_25_,
            act_type='relu',
            name="relu16_1_25_")

        conv17_1_25_ = mx.symbol.Convolution(data=relu16_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_25_")
        # conv17_1_25_, output shape: {[256,56,56]}

        batchnorm17_1_25_ = mx.symbol.BatchNorm(data=conv17_1_25_,
            fix_gamma=True,
            name="batchnorm17_1_25_")
        conv15_1_26_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_26_")
        # conv15_1_26_, output shape: {[4,56,56]}

        batchnorm15_1_26_ = mx.symbol.BatchNorm(data=conv15_1_26_,
            fix_gamma=True,
            name="batchnorm15_1_26_")
        relu15_1_26_ = mx.symbol.Activation(data=batchnorm15_1_26_,
            act_type='relu',
            name="relu15_1_26_")

        conv16_1_26_ = mx.symbol.pad(data=relu15_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_26_ = mx.symbol.Convolution(data=conv16_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_26_")
        # conv16_1_26_, output shape: {[4,56,56]}

        batchnorm16_1_26_ = mx.symbol.BatchNorm(data=conv16_1_26_,
            fix_gamma=True,
            name="batchnorm16_1_26_")
        relu16_1_26_ = mx.symbol.Activation(data=batchnorm16_1_26_,
            act_type='relu',
            name="relu16_1_26_")

        conv17_1_26_ = mx.symbol.Convolution(data=relu16_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_26_")
        # conv17_1_26_, output shape: {[256,56,56]}

        batchnorm17_1_26_ = mx.symbol.BatchNorm(data=conv17_1_26_,
            fix_gamma=True,
            name="batchnorm17_1_26_")
        conv15_1_27_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_27_")
        # conv15_1_27_, output shape: {[4,56,56]}

        batchnorm15_1_27_ = mx.symbol.BatchNorm(data=conv15_1_27_,
            fix_gamma=True,
            name="batchnorm15_1_27_")
        relu15_1_27_ = mx.symbol.Activation(data=batchnorm15_1_27_,
            act_type='relu',
            name="relu15_1_27_")

        conv16_1_27_ = mx.symbol.pad(data=relu15_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_27_ = mx.symbol.Convolution(data=conv16_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_27_")
        # conv16_1_27_, output shape: {[4,56,56]}

        batchnorm16_1_27_ = mx.symbol.BatchNorm(data=conv16_1_27_,
            fix_gamma=True,
            name="batchnorm16_1_27_")
        relu16_1_27_ = mx.symbol.Activation(data=batchnorm16_1_27_,
            act_type='relu',
            name="relu16_1_27_")

        conv17_1_27_ = mx.symbol.Convolution(data=relu16_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_27_")
        # conv17_1_27_, output shape: {[256,56,56]}

        batchnorm17_1_27_ = mx.symbol.BatchNorm(data=conv17_1_27_,
            fix_gamma=True,
            name="batchnorm17_1_27_")
        conv15_1_28_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_28_")
        # conv15_1_28_, output shape: {[4,56,56]}

        batchnorm15_1_28_ = mx.symbol.BatchNorm(data=conv15_1_28_,
            fix_gamma=True,
            name="batchnorm15_1_28_")
        relu15_1_28_ = mx.symbol.Activation(data=batchnorm15_1_28_,
            act_type='relu',
            name="relu15_1_28_")

        conv16_1_28_ = mx.symbol.pad(data=relu15_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_28_ = mx.symbol.Convolution(data=conv16_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_28_")
        # conv16_1_28_, output shape: {[4,56,56]}

        batchnorm16_1_28_ = mx.symbol.BatchNorm(data=conv16_1_28_,
            fix_gamma=True,
            name="batchnorm16_1_28_")
        relu16_1_28_ = mx.symbol.Activation(data=batchnorm16_1_28_,
            act_type='relu',
            name="relu16_1_28_")

        conv17_1_28_ = mx.symbol.Convolution(data=relu16_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_28_")
        # conv17_1_28_, output shape: {[256,56,56]}

        batchnorm17_1_28_ = mx.symbol.BatchNorm(data=conv17_1_28_,
            fix_gamma=True,
            name="batchnorm17_1_28_")
        conv15_1_29_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_29_")
        # conv15_1_29_, output shape: {[4,56,56]}

        batchnorm15_1_29_ = mx.symbol.BatchNorm(data=conv15_1_29_,
            fix_gamma=True,
            name="batchnorm15_1_29_")
        relu15_1_29_ = mx.symbol.Activation(data=batchnorm15_1_29_,
            act_type='relu',
            name="relu15_1_29_")

        conv16_1_29_ = mx.symbol.pad(data=relu15_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_29_ = mx.symbol.Convolution(data=conv16_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_29_")
        # conv16_1_29_, output shape: {[4,56,56]}

        batchnorm16_1_29_ = mx.symbol.BatchNorm(data=conv16_1_29_,
            fix_gamma=True,
            name="batchnorm16_1_29_")
        relu16_1_29_ = mx.symbol.Activation(data=batchnorm16_1_29_,
            act_type='relu',
            name="relu16_1_29_")

        conv17_1_29_ = mx.symbol.Convolution(data=relu16_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_29_")
        # conv17_1_29_, output shape: {[256,56,56]}

        batchnorm17_1_29_ = mx.symbol.BatchNorm(data=conv17_1_29_,
            fix_gamma=True,
            name="batchnorm17_1_29_")
        conv15_1_30_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_30_")
        # conv15_1_30_, output shape: {[4,56,56]}

        batchnorm15_1_30_ = mx.symbol.BatchNorm(data=conv15_1_30_,
            fix_gamma=True,
            name="batchnorm15_1_30_")
        relu15_1_30_ = mx.symbol.Activation(data=batchnorm15_1_30_,
            act_type='relu',
            name="relu15_1_30_")

        conv16_1_30_ = mx.symbol.pad(data=relu15_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_30_ = mx.symbol.Convolution(data=conv16_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_30_")
        # conv16_1_30_, output shape: {[4,56,56]}

        batchnorm16_1_30_ = mx.symbol.BatchNorm(data=conv16_1_30_,
            fix_gamma=True,
            name="batchnorm16_1_30_")
        relu16_1_30_ = mx.symbol.Activation(data=batchnorm16_1_30_,
            act_type='relu',
            name="relu16_1_30_")

        conv17_1_30_ = mx.symbol.Convolution(data=relu16_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_30_")
        # conv17_1_30_, output shape: {[256,56,56]}

        batchnorm17_1_30_ = mx.symbol.BatchNorm(data=conv17_1_30_,
            fix_gamma=True,
            name="batchnorm17_1_30_")
        conv15_1_31_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_31_")
        # conv15_1_31_, output shape: {[4,56,56]}

        batchnorm15_1_31_ = mx.symbol.BatchNorm(data=conv15_1_31_,
            fix_gamma=True,
            name="batchnorm15_1_31_")
        relu15_1_31_ = mx.symbol.Activation(data=batchnorm15_1_31_,
            act_type='relu',
            name="relu15_1_31_")

        conv16_1_31_ = mx.symbol.pad(data=relu15_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_31_ = mx.symbol.Convolution(data=conv16_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_31_")
        # conv16_1_31_, output shape: {[4,56,56]}

        batchnorm16_1_31_ = mx.symbol.BatchNorm(data=conv16_1_31_,
            fix_gamma=True,
            name="batchnorm16_1_31_")
        relu16_1_31_ = mx.symbol.Activation(data=batchnorm16_1_31_,
            act_type='relu',
            name="relu16_1_31_")

        conv17_1_31_ = mx.symbol.Convolution(data=relu16_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_31_")
        # conv17_1_31_, output shape: {[256,56,56]}

        batchnorm17_1_31_ = mx.symbol.BatchNorm(data=conv17_1_31_,
            fix_gamma=True,
            name="batchnorm17_1_31_")
        conv15_1_32_ = mx.symbol.Convolution(data=relu13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv15_1_32_")
        # conv15_1_32_, output shape: {[4,56,56]}

        batchnorm15_1_32_ = mx.symbol.BatchNorm(data=conv15_1_32_,
            fix_gamma=True,
            name="batchnorm15_1_32_")
        relu15_1_32_ = mx.symbol.Activation(data=batchnorm15_1_32_,
            act_type='relu',
            name="relu15_1_32_")

        conv16_1_32_ = mx.symbol.pad(data=relu15_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv16_1_32_ = mx.symbol.Convolution(data=conv16_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=4,
            no_bias=False,
            name="conv16_1_32_")
        # conv16_1_32_, output shape: {[4,56,56]}

        batchnorm16_1_32_ = mx.symbol.BatchNorm(data=conv16_1_32_,
            fix_gamma=True,
            name="batchnorm16_1_32_")
        relu16_1_32_ = mx.symbol.Activation(data=batchnorm16_1_32_,
            act_type='relu',
            name="relu16_1_32_")

        conv17_1_32_ = mx.symbol.Convolution(data=relu16_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv17_1_32_")
        # conv17_1_32_, output shape: {[256,56,56]}

        batchnorm17_1_32_ = mx.symbol.BatchNorm(data=conv17_1_32_,
            fix_gamma=True,
            name="batchnorm17_1_32_")
        add18_1_ = batchnorm17_1_1_ + batchnorm17_1_2_ + batchnorm17_1_3_ + batchnorm17_1_4_ + batchnorm17_1_5_ + batchnorm17_1_6_ + batchnorm17_1_7_ + batchnorm17_1_8_ + batchnorm17_1_9_ + batchnorm17_1_10_ + batchnorm17_1_11_ + batchnorm17_1_12_ + batchnorm17_1_13_ + batchnorm17_1_14_ + batchnorm17_1_15_ + batchnorm17_1_16_ + batchnorm17_1_17_ + batchnorm17_1_18_ + batchnorm17_1_19_ + batchnorm17_1_20_ + batchnorm17_1_21_ + batchnorm17_1_22_ + batchnorm17_1_23_ + batchnorm17_1_24_ + batchnorm17_1_25_ + batchnorm17_1_26_ + batchnorm17_1_27_ + batchnorm17_1_28_ + batchnorm17_1_29_ + batchnorm17_1_30_ + batchnorm17_1_31_ + batchnorm17_1_32_
        # add18_1_, output shape: {[256,56,56]}

        add19_ = add18_1_ + relu13_
        # add19_, output shape: {[256,56,56]}

        relu19_ = mx.symbol.Activation(data=add19_,
            act_type='relu',
            name="relu19_")

        conv21_1_1_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_1_")
        # conv21_1_1_, output shape: {[8,56,56]}

        batchnorm21_1_1_ = mx.symbol.BatchNorm(data=conv21_1_1_,
            fix_gamma=True,
            name="batchnorm21_1_1_")
        relu21_1_1_ = mx.symbol.Activation(data=batchnorm21_1_1_,
            act_type='relu',
            name="relu21_1_1_")

        conv22_1_1_ = mx.symbol.pad(data=relu21_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_1_ = mx.symbol.Convolution(data=conv22_1_1_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_1_")
        # conv22_1_1_, output shape: {[8,28,28]}

        batchnorm22_1_1_ = mx.symbol.BatchNorm(data=conv22_1_1_,
            fix_gamma=True,
            name="batchnorm22_1_1_")
        relu22_1_1_ = mx.symbol.Activation(data=batchnorm22_1_1_,
            act_type='relu',
            name="relu22_1_1_")

        conv23_1_1_ = mx.symbol.Convolution(data=relu22_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_1_")
        # conv23_1_1_, output shape: {[512,28,28]}

        batchnorm23_1_1_ = mx.symbol.BatchNorm(data=conv23_1_1_,
            fix_gamma=True,
            name="batchnorm23_1_1_")
        conv21_1_2_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_2_")
        # conv21_1_2_, output shape: {[8,56,56]}

        batchnorm21_1_2_ = mx.symbol.BatchNorm(data=conv21_1_2_,
            fix_gamma=True,
            name="batchnorm21_1_2_")
        relu21_1_2_ = mx.symbol.Activation(data=batchnorm21_1_2_,
            act_type='relu',
            name="relu21_1_2_")

        conv22_1_2_ = mx.symbol.pad(data=relu21_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_2_ = mx.symbol.Convolution(data=conv22_1_2_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_2_")
        # conv22_1_2_, output shape: {[8,28,28]}

        batchnorm22_1_2_ = mx.symbol.BatchNorm(data=conv22_1_2_,
            fix_gamma=True,
            name="batchnorm22_1_2_")
        relu22_1_2_ = mx.symbol.Activation(data=batchnorm22_1_2_,
            act_type='relu',
            name="relu22_1_2_")

        conv23_1_2_ = mx.symbol.Convolution(data=relu22_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_2_")
        # conv23_1_2_, output shape: {[512,28,28]}

        batchnorm23_1_2_ = mx.symbol.BatchNorm(data=conv23_1_2_,
            fix_gamma=True,
            name="batchnorm23_1_2_")
        conv21_1_3_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_3_")
        # conv21_1_3_, output shape: {[8,56,56]}

        batchnorm21_1_3_ = mx.symbol.BatchNorm(data=conv21_1_3_,
            fix_gamma=True,
            name="batchnorm21_1_3_")
        relu21_1_3_ = mx.symbol.Activation(data=batchnorm21_1_3_,
            act_type='relu',
            name="relu21_1_3_")

        conv22_1_3_ = mx.symbol.pad(data=relu21_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_3_ = mx.symbol.Convolution(data=conv22_1_3_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_3_")
        # conv22_1_3_, output shape: {[8,28,28]}

        batchnorm22_1_3_ = mx.symbol.BatchNorm(data=conv22_1_3_,
            fix_gamma=True,
            name="batchnorm22_1_3_")
        relu22_1_3_ = mx.symbol.Activation(data=batchnorm22_1_3_,
            act_type='relu',
            name="relu22_1_3_")

        conv23_1_3_ = mx.symbol.Convolution(data=relu22_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_3_")
        # conv23_1_3_, output shape: {[512,28,28]}

        batchnorm23_1_3_ = mx.symbol.BatchNorm(data=conv23_1_3_,
            fix_gamma=True,
            name="batchnorm23_1_3_")
        conv21_1_4_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_4_")
        # conv21_1_4_, output shape: {[8,56,56]}

        batchnorm21_1_4_ = mx.symbol.BatchNorm(data=conv21_1_4_,
            fix_gamma=True,
            name="batchnorm21_1_4_")
        relu21_1_4_ = mx.symbol.Activation(data=batchnorm21_1_4_,
            act_type='relu',
            name="relu21_1_4_")

        conv22_1_4_ = mx.symbol.pad(data=relu21_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_4_ = mx.symbol.Convolution(data=conv22_1_4_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_4_")
        # conv22_1_4_, output shape: {[8,28,28]}

        batchnorm22_1_4_ = mx.symbol.BatchNorm(data=conv22_1_4_,
            fix_gamma=True,
            name="batchnorm22_1_4_")
        relu22_1_4_ = mx.symbol.Activation(data=batchnorm22_1_4_,
            act_type='relu',
            name="relu22_1_4_")

        conv23_1_4_ = mx.symbol.Convolution(data=relu22_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_4_")
        # conv23_1_4_, output shape: {[512,28,28]}

        batchnorm23_1_4_ = mx.symbol.BatchNorm(data=conv23_1_4_,
            fix_gamma=True,
            name="batchnorm23_1_4_")
        conv21_1_5_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_5_")
        # conv21_1_5_, output shape: {[8,56,56]}

        batchnorm21_1_5_ = mx.symbol.BatchNorm(data=conv21_1_5_,
            fix_gamma=True,
            name="batchnorm21_1_5_")
        relu21_1_5_ = mx.symbol.Activation(data=batchnorm21_1_5_,
            act_type='relu',
            name="relu21_1_5_")

        conv22_1_5_ = mx.symbol.pad(data=relu21_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_5_ = mx.symbol.Convolution(data=conv22_1_5_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_5_")
        # conv22_1_5_, output shape: {[8,28,28]}

        batchnorm22_1_5_ = mx.symbol.BatchNorm(data=conv22_1_5_,
            fix_gamma=True,
            name="batchnorm22_1_5_")
        relu22_1_5_ = mx.symbol.Activation(data=batchnorm22_1_5_,
            act_type='relu',
            name="relu22_1_5_")

        conv23_1_5_ = mx.symbol.Convolution(data=relu22_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_5_")
        # conv23_1_5_, output shape: {[512,28,28]}

        batchnorm23_1_5_ = mx.symbol.BatchNorm(data=conv23_1_5_,
            fix_gamma=True,
            name="batchnorm23_1_5_")
        conv21_1_6_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_6_")
        # conv21_1_6_, output shape: {[8,56,56]}

        batchnorm21_1_6_ = mx.symbol.BatchNorm(data=conv21_1_6_,
            fix_gamma=True,
            name="batchnorm21_1_6_")
        relu21_1_6_ = mx.symbol.Activation(data=batchnorm21_1_6_,
            act_type='relu',
            name="relu21_1_6_")

        conv22_1_6_ = mx.symbol.pad(data=relu21_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_6_ = mx.symbol.Convolution(data=conv22_1_6_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_6_")
        # conv22_1_6_, output shape: {[8,28,28]}

        batchnorm22_1_6_ = mx.symbol.BatchNorm(data=conv22_1_6_,
            fix_gamma=True,
            name="batchnorm22_1_6_")
        relu22_1_6_ = mx.symbol.Activation(data=batchnorm22_1_6_,
            act_type='relu',
            name="relu22_1_6_")

        conv23_1_6_ = mx.symbol.Convolution(data=relu22_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_6_")
        # conv23_1_6_, output shape: {[512,28,28]}

        batchnorm23_1_6_ = mx.symbol.BatchNorm(data=conv23_1_6_,
            fix_gamma=True,
            name="batchnorm23_1_6_")
        conv21_1_7_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_7_")
        # conv21_1_7_, output shape: {[8,56,56]}

        batchnorm21_1_7_ = mx.symbol.BatchNorm(data=conv21_1_7_,
            fix_gamma=True,
            name="batchnorm21_1_7_")
        relu21_1_7_ = mx.symbol.Activation(data=batchnorm21_1_7_,
            act_type='relu',
            name="relu21_1_7_")

        conv22_1_7_ = mx.symbol.pad(data=relu21_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_7_ = mx.symbol.Convolution(data=conv22_1_7_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_7_")
        # conv22_1_7_, output shape: {[8,28,28]}

        batchnorm22_1_7_ = mx.symbol.BatchNorm(data=conv22_1_7_,
            fix_gamma=True,
            name="batchnorm22_1_7_")
        relu22_1_7_ = mx.symbol.Activation(data=batchnorm22_1_7_,
            act_type='relu',
            name="relu22_1_7_")

        conv23_1_7_ = mx.symbol.Convolution(data=relu22_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_7_")
        # conv23_1_7_, output shape: {[512,28,28]}

        batchnorm23_1_7_ = mx.symbol.BatchNorm(data=conv23_1_7_,
            fix_gamma=True,
            name="batchnorm23_1_7_")
        conv21_1_8_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_8_")
        # conv21_1_8_, output shape: {[8,56,56]}

        batchnorm21_1_8_ = mx.symbol.BatchNorm(data=conv21_1_8_,
            fix_gamma=True,
            name="batchnorm21_1_8_")
        relu21_1_8_ = mx.symbol.Activation(data=batchnorm21_1_8_,
            act_type='relu',
            name="relu21_1_8_")

        conv22_1_8_ = mx.symbol.pad(data=relu21_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_8_ = mx.symbol.Convolution(data=conv22_1_8_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_8_")
        # conv22_1_8_, output shape: {[8,28,28]}

        batchnorm22_1_8_ = mx.symbol.BatchNorm(data=conv22_1_8_,
            fix_gamma=True,
            name="batchnorm22_1_8_")
        relu22_1_8_ = mx.symbol.Activation(data=batchnorm22_1_8_,
            act_type='relu',
            name="relu22_1_8_")

        conv23_1_8_ = mx.symbol.Convolution(data=relu22_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_8_")
        # conv23_1_8_, output shape: {[512,28,28]}

        batchnorm23_1_8_ = mx.symbol.BatchNorm(data=conv23_1_8_,
            fix_gamma=True,
            name="batchnorm23_1_8_")
        conv21_1_9_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_9_")
        # conv21_1_9_, output shape: {[8,56,56]}

        batchnorm21_1_9_ = mx.symbol.BatchNorm(data=conv21_1_9_,
            fix_gamma=True,
            name="batchnorm21_1_9_")
        relu21_1_9_ = mx.symbol.Activation(data=batchnorm21_1_9_,
            act_type='relu',
            name="relu21_1_9_")

        conv22_1_9_ = mx.symbol.pad(data=relu21_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_9_ = mx.symbol.Convolution(data=conv22_1_9_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_9_")
        # conv22_1_9_, output shape: {[8,28,28]}

        batchnorm22_1_9_ = mx.symbol.BatchNorm(data=conv22_1_9_,
            fix_gamma=True,
            name="batchnorm22_1_9_")
        relu22_1_9_ = mx.symbol.Activation(data=batchnorm22_1_9_,
            act_type='relu',
            name="relu22_1_9_")

        conv23_1_9_ = mx.symbol.Convolution(data=relu22_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_9_")
        # conv23_1_9_, output shape: {[512,28,28]}

        batchnorm23_1_9_ = mx.symbol.BatchNorm(data=conv23_1_9_,
            fix_gamma=True,
            name="batchnorm23_1_9_")
        conv21_1_10_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_10_")
        # conv21_1_10_, output shape: {[8,56,56]}

        batchnorm21_1_10_ = mx.symbol.BatchNorm(data=conv21_1_10_,
            fix_gamma=True,
            name="batchnorm21_1_10_")
        relu21_1_10_ = mx.symbol.Activation(data=batchnorm21_1_10_,
            act_type='relu',
            name="relu21_1_10_")

        conv22_1_10_ = mx.symbol.pad(data=relu21_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_10_ = mx.symbol.Convolution(data=conv22_1_10_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_10_")
        # conv22_1_10_, output shape: {[8,28,28]}

        batchnorm22_1_10_ = mx.symbol.BatchNorm(data=conv22_1_10_,
            fix_gamma=True,
            name="batchnorm22_1_10_")
        relu22_1_10_ = mx.symbol.Activation(data=batchnorm22_1_10_,
            act_type='relu',
            name="relu22_1_10_")

        conv23_1_10_ = mx.symbol.Convolution(data=relu22_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_10_")
        # conv23_1_10_, output shape: {[512,28,28]}

        batchnorm23_1_10_ = mx.symbol.BatchNorm(data=conv23_1_10_,
            fix_gamma=True,
            name="batchnorm23_1_10_")
        conv21_1_11_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_11_")
        # conv21_1_11_, output shape: {[8,56,56]}

        batchnorm21_1_11_ = mx.symbol.BatchNorm(data=conv21_1_11_,
            fix_gamma=True,
            name="batchnorm21_1_11_")
        relu21_1_11_ = mx.symbol.Activation(data=batchnorm21_1_11_,
            act_type='relu',
            name="relu21_1_11_")

        conv22_1_11_ = mx.symbol.pad(data=relu21_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_11_ = mx.symbol.Convolution(data=conv22_1_11_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_11_")
        # conv22_1_11_, output shape: {[8,28,28]}

        batchnorm22_1_11_ = mx.symbol.BatchNorm(data=conv22_1_11_,
            fix_gamma=True,
            name="batchnorm22_1_11_")
        relu22_1_11_ = mx.symbol.Activation(data=batchnorm22_1_11_,
            act_type='relu',
            name="relu22_1_11_")

        conv23_1_11_ = mx.symbol.Convolution(data=relu22_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_11_")
        # conv23_1_11_, output shape: {[512,28,28]}

        batchnorm23_1_11_ = mx.symbol.BatchNorm(data=conv23_1_11_,
            fix_gamma=True,
            name="batchnorm23_1_11_")
        conv21_1_12_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_12_")
        # conv21_1_12_, output shape: {[8,56,56]}

        batchnorm21_1_12_ = mx.symbol.BatchNorm(data=conv21_1_12_,
            fix_gamma=True,
            name="batchnorm21_1_12_")
        relu21_1_12_ = mx.symbol.Activation(data=batchnorm21_1_12_,
            act_type='relu',
            name="relu21_1_12_")

        conv22_1_12_ = mx.symbol.pad(data=relu21_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_12_ = mx.symbol.Convolution(data=conv22_1_12_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_12_")
        # conv22_1_12_, output shape: {[8,28,28]}

        batchnorm22_1_12_ = mx.symbol.BatchNorm(data=conv22_1_12_,
            fix_gamma=True,
            name="batchnorm22_1_12_")
        relu22_1_12_ = mx.symbol.Activation(data=batchnorm22_1_12_,
            act_type='relu',
            name="relu22_1_12_")

        conv23_1_12_ = mx.symbol.Convolution(data=relu22_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_12_")
        # conv23_1_12_, output shape: {[512,28,28]}

        batchnorm23_1_12_ = mx.symbol.BatchNorm(data=conv23_1_12_,
            fix_gamma=True,
            name="batchnorm23_1_12_")
        conv21_1_13_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_13_")
        # conv21_1_13_, output shape: {[8,56,56]}

        batchnorm21_1_13_ = mx.symbol.BatchNorm(data=conv21_1_13_,
            fix_gamma=True,
            name="batchnorm21_1_13_")
        relu21_1_13_ = mx.symbol.Activation(data=batchnorm21_1_13_,
            act_type='relu',
            name="relu21_1_13_")

        conv22_1_13_ = mx.symbol.pad(data=relu21_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_13_ = mx.symbol.Convolution(data=conv22_1_13_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_13_")
        # conv22_1_13_, output shape: {[8,28,28]}

        batchnorm22_1_13_ = mx.symbol.BatchNorm(data=conv22_1_13_,
            fix_gamma=True,
            name="batchnorm22_1_13_")
        relu22_1_13_ = mx.symbol.Activation(data=batchnorm22_1_13_,
            act_type='relu',
            name="relu22_1_13_")

        conv23_1_13_ = mx.symbol.Convolution(data=relu22_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_13_")
        # conv23_1_13_, output shape: {[512,28,28]}

        batchnorm23_1_13_ = mx.symbol.BatchNorm(data=conv23_1_13_,
            fix_gamma=True,
            name="batchnorm23_1_13_")
        conv21_1_14_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_14_")
        # conv21_1_14_, output shape: {[8,56,56]}

        batchnorm21_1_14_ = mx.symbol.BatchNorm(data=conv21_1_14_,
            fix_gamma=True,
            name="batchnorm21_1_14_")
        relu21_1_14_ = mx.symbol.Activation(data=batchnorm21_1_14_,
            act_type='relu',
            name="relu21_1_14_")

        conv22_1_14_ = mx.symbol.pad(data=relu21_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_14_ = mx.symbol.Convolution(data=conv22_1_14_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_14_")
        # conv22_1_14_, output shape: {[8,28,28]}

        batchnorm22_1_14_ = mx.symbol.BatchNorm(data=conv22_1_14_,
            fix_gamma=True,
            name="batchnorm22_1_14_")
        relu22_1_14_ = mx.symbol.Activation(data=batchnorm22_1_14_,
            act_type='relu',
            name="relu22_1_14_")

        conv23_1_14_ = mx.symbol.Convolution(data=relu22_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_14_")
        # conv23_1_14_, output shape: {[512,28,28]}

        batchnorm23_1_14_ = mx.symbol.BatchNorm(data=conv23_1_14_,
            fix_gamma=True,
            name="batchnorm23_1_14_")
        conv21_1_15_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_15_")
        # conv21_1_15_, output shape: {[8,56,56]}

        batchnorm21_1_15_ = mx.symbol.BatchNorm(data=conv21_1_15_,
            fix_gamma=True,
            name="batchnorm21_1_15_")
        relu21_1_15_ = mx.symbol.Activation(data=batchnorm21_1_15_,
            act_type='relu',
            name="relu21_1_15_")

        conv22_1_15_ = mx.symbol.pad(data=relu21_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_15_ = mx.symbol.Convolution(data=conv22_1_15_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_15_")
        # conv22_1_15_, output shape: {[8,28,28]}

        batchnorm22_1_15_ = mx.symbol.BatchNorm(data=conv22_1_15_,
            fix_gamma=True,
            name="batchnorm22_1_15_")
        relu22_1_15_ = mx.symbol.Activation(data=batchnorm22_1_15_,
            act_type='relu',
            name="relu22_1_15_")

        conv23_1_15_ = mx.symbol.Convolution(data=relu22_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_15_")
        # conv23_1_15_, output shape: {[512,28,28]}

        batchnorm23_1_15_ = mx.symbol.BatchNorm(data=conv23_1_15_,
            fix_gamma=True,
            name="batchnorm23_1_15_")
        conv21_1_16_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_16_")
        # conv21_1_16_, output shape: {[8,56,56]}

        batchnorm21_1_16_ = mx.symbol.BatchNorm(data=conv21_1_16_,
            fix_gamma=True,
            name="batchnorm21_1_16_")
        relu21_1_16_ = mx.symbol.Activation(data=batchnorm21_1_16_,
            act_type='relu',
            name="relu21_1_16_")

        conv22_1_16_ = mx.symbol.pad(data=relu21_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_16_ = mx.symbol.Convolution(data=conv22_1_16_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_16_")
        # conv22_1_16_, output shape: {[8,28,28]}

        batchnorm22_1_16_ = mx.symbol.BatchNorm(data=conv22_1_16_,
            fix_gamma=True,
            name="batchnorm22_1_16_")
        relu22_1_16_ = mx.symbol.Activation(data=batchnorm22_1_16_,
            act_type='relu',
            name="relu22_1_16_")

        conv23_1_16_ = mx.symbol.Convolution(data=relu22_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_16_")
        # conv23_1_16_, output shape: {[512,28,28]}

        batchnorm23_1_16_ = mx.symbol.BatchNorm(data=conv23_1_16_,
            fix_gamma=True,
            name="batchnorm23_1_16_")
        conv21_1_17_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_17_")
        # conv21_1_17_, output shape: {[8,56,56]}

        batchnorm21_1_17_ = mx.symbol.BatchNorm(data=conv21_1_17_,
            fix_gamma=True,
            name="batchnorm21_1_17_")
        relu21_1_17_ = mx.symbol.Activation(data=batchnorm21_1_17_,
            act_type='relu',
            name="relu21_1_17_")

        conv22_1_17_ = mx.symbol.pad(data=relu21_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_17_ = mx.symbol.Convolution(data=conv22_1_17_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_17_")
        # conv22_1_17_, output shape: {[8,28,28]}

        batchnorm22_1_17_ = mx.symbol.BatchNorm(data=conv22_1_17_,
            fix_gamma=True,
            name="batchnorm22_1_17_")
        relu22_1_17_ = mx.symbol.Activation(data=batchnorm22_1_17_,
            act_type='relu',
            name="relu22_1_17_")

        conv23_1_17_ = mx.symbol.Convolution(data=relu22_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_17_")
        # conv23_1_17_, output shape: {[512,28,28]}

        batchnorm23_1_17_ = mx.symbol.BatchNorm(data=conv23_1_17_,
            fix_gamma=True,
            name="batchnorm23_1_17_")
        conv21_1_18_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_18_")
        # conv21_1_18_, output shape: {[8,56,56]}

        batchnorm21_1_18_ = mx.symbol.BatchNorm(data=conv21_1_18_,
            fix_gamma=True,
            name="batchnorm21_1_18_")
        relu21_1_18_ = mx.symbol.Activation(data=batchnorm21_1_18_,
            act_type='relu',
            name="relu21_1_18_")

        conv22_1_18_ = mx.symbol.pad(data=relu21_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_18_ = mx.symbol.Convolution(data=conv22_1_18_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_18_")
        # conv22_1_18_, output shape: {[8,28,28]}

        batchnorm22_1_18_ = mx.symbol.BatchNorm(data=conv22_1_18_,
            fix_gamma=True,
            name="batchnorm22_1_18_")
        relu22_1_18_ = mx.symbol.Activation(data=batchnorm22_1_18_,
            act_type='relu',
            name="relu22_1_18_")

        conv23_1_18_ = mx.symbol.Convolution(data=relu22_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_18_")
        # conv23_1_18_, output shape: {[512,28,28]}

        batchnorm23_1_18_ = mx.symbol.BatchNorm(data=conv23_1_18_,
            fix_gamma=True,
            name="batchnorm23_1_18_")
        conv21_1_19_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_19_")
        # conv21_1_19_, output shape: {[8,56,56]}

        batchnorm21_1_19_ = mx.symbol.BatchNorm(data=conv21_1_19_,
            fix_gamma=True,
            name="batchnorm21_1_19_")
        relu21_1_19_ = mx.symbol.Activation(data=batchnorm21_1_19_,
            act_type='relu',
            name="relu21_1_19_")

        conv22_1_19_ = mx.symbol.pad(data=relu21_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_19_ = mx.symbol.Convolution(data=conv22_1_19_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_19_")
        # conv22_1_19_, output shape: {[8,28,28]}

        batchnorm22_1_19_ = mx.symbol.BatchNorm(data=conv22_1_19_,
            fix_gamma=True,
            name="batchnorm22_1_19_")
        relu22_1_19_ = mx.symbol.Activation(data=batchnorm22_1_19_,
            act_type='relu',
            name="relu22_1_19_")

        conv23_1_19_ = mx.symbol.Convolution(data=relu22_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_19_")
        # conv23_1_19_, output shape: {[512,28,28]}

        batchnorm23_1_19_ = mx.symbol.BatchNorm(data=conv23_1_19_,
            fix_gamma=True,
            name="batchnorm23_1_19_")
        conv21_1_20_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_20_")
        # conv21_1_20_, output shape: {[8,56,56]}

        batchnorm21_1_20_ = mx.symbol.BatchNorm(data=conv21_1_20_,
            fix_gamma=True,
            name="batchnorm21_1_20_")
        relu21_1_20_ = mx.symbol.Activation(data=batchnorm21_1_20_,
            act_type='relu',
            name="relu21_1_20_")

        conv22_1_20_ = mx.symbol.pad(data=relu21_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_20_ = mx.symbol.Convolution(data=conv22_1_20_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_20_")
        # conv22_1_20_, output shape: {[8,28,28]}

        batchnorm22_1_20_ = mx.symbol.BatchNorm(data=conv22_1_20_,
            fix_gamma=True,
            name="batchnorm22_1_20_")
        relu22_1_20_ = mx.symbol.Activation(data=batchnorm22_1_20_,
            act_type='relu',
            name="relu22_1_20_")

        conv23_1_20_ = mx.symbol.Convolution(data=relu22_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_20_")
        # conv23_1_20_, output shape: {[512,28,28]}

        batchnorm23_1_20_ = mx.symbol.BatchNorm(data=conv23_1_20_,
            fix_gamma=True,
            name="batchnorm23_1_20_")
        conv21_1_21_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_21_")
        # conv21_1_21_, output shape: {[8,56,56]}

        batchnorm21_1_21_ = mx.symbol.BatchNorm(data=conv21_1_21_,
            fix_gamma=True,
            name="batchnorm21_1_21_")
        relu21_1_21_ = mx.symbol.Activation(data=batchnorm21_1_21_,
            act_type='relu',
            name="relu21_1_21_")

        conv22_1_21_ = mx.symbol.pad(data=relu21_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_21_ = mx.symbol.Convolution(data=conv22_1_21_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_21_")
        # conv22_1_21_, output shape: {[8,28,28]}

        batchnorm22_1_21_ = mx.symbol.BatchNorm(data=conv22_1_21_,
            fix_gamma=True,
            name="batchnorm22_1_21_")
        relu22_1_21_ = mx.symbol.Activation(data=batchnorm22_1_21_,
            act_type='relu',
            name="relu22_1_21_")

        conv23_1_21_ = mx.symbol.Convolution(data=relu22_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_21_")
        # conv23_1_21_, output shape: {[512,28,28]}

        batchnorm23_1_21_ = mx.symbol.BatchNorm(data=conv23_1_21_,
            fix_gamma=True,
            name="batchnorm23_1_21_")
        conv21_1_22_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_22_")
        # conv21_1_22_, output shape: {[8,56,56]}

        batchnorm21_1_22_ = mx.symbol.BatchNorm(data=conv21_1_22_,
            fix_gamma=True,
            name="batchnorm21_1_22_")
        relu21_1_22_ = mx.symbol.Activation(data=batchnorm21_1_22_,
            act_type='relu',
            name="relu21_1_22_")

        conv22_1_22_ = mx.symbol.pad(data=relu21_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_22_ = mx.symbol.Convolution(data=conv22_1_22_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_22_")
        # conv22_1_22_, output shape: {[8,28,28]}

        batchnorm22_1_22_ = mx.symbol.BatchNorm(data=conv22_1_22_,
            fix_gamma=True,
            name="batchnorm22_1_22_")
        relu22_1_22_ = mx.symbol.Activation(data=batchnorm22_1_22_,
            act_type='relu',
            name="relu22_1_22_")

        conv23_1_22_ = mx.symbol.Convolution(data=relu22_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_22_")
        # conv23_1_22_, output shape: {[512,28,28]}

        batchnorm23_1_22_ = mx.symbol.BatchNorm(data=conv23_1_22_,
            fix_gamma=True,
            name="batchnorm23_1_22_")
        conv21_1_23_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_23_")
        # conv21_1_23_, output shape: {[8,56,56]}

        batchnorm21_1_23_ = mx.symbol.BatchNorm(data=conv21_1_23_,
            fix_gamma=True,
            name="batchnorm21_1_23_")
        relu21_1_23_ = mx.symbol.Activation(data=batchnorm21_1_23_,
            act_type='relu',
            name="relu21_1_23_")

        conv22_1_23_ = mx.symbol.pad(data=relu21_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_23_ = mx.symbol.Convolution(data=conv22_1_23_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_23_")
        # conv22_1_23_, output shape: {[8,28,28]}

        batchnorm22_1_23_ = mx.symbol.BatchNorm(data=conv22_1_23_,
            fix_gamma=True,
            name="batchnorm22_1_23_")
        relu22_1_23_ = mx.symbol.Activation(data=batchnorm22_1_23_,
            act_type='relu',
            name="relu22_1_23_")

        conv23_1_23_ = mx.symbol.Convolution(data=relu22_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_23_")
        # conv23_1_23_, output shape: {[512,28,28]}

        batchnorm23_1_23_ = mx.symbol.BatchNorm(data=conv23_1_23_,
            fix_gamma=True,
            name="batchnorm23_1_23_")
        conv21_1_24_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_24_")
        # conv21_1_24_, output shape: {[8,56,56]}

        batchnorm21_1_24_ = mx.symbol.BatchNorm(data=conv21_1_24_,
            fix_gamma=True,
            name="batchnorm21_1_24_")
        relu21_1_24_ = mx.symbol.Activation(data=batchnorm21_1_24_,
            act_type='relu',
            name="relu21_1_24_")

        conv22_1_24_ = mx.symbol.pad(data=relu21_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_24_ = mx.symbol.Convolution(data=conv22_1_24_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_24_")
        # conv22_1_24_, output shape: {[8,28,28]}

        batchnorm22_1_24_ = mx.symbol.BatchNorm(data=conv22_1_24_,
            fix_gamma=True,
            name="batchnorm22_1_24_")
        relu22_1_24_ = mx.symbol.Activation(data=batchnorm22_1_24_,
            act_type='relu',
            name="relu22_1_24_")

        conv23_1_24_ = mx.symbol.Convolution(data=relu22_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_24_")
        # conv23_1_24_, output shape: {[512,28,28]}

        batchnorm23_1_24_ = mx.symbol.BatchNorm(data=conv23_1_24_,
            fix_gamma=True,
            name="batchnorm23_1_24_")
        conv21_1_25_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_25_")
        # conv21_1_25_, output shape: {[8,56,56]}

        batchnorm21_1_25_ = mx.symbol.BatchNorm(data=conv21_1_25_,
            fix_gamma=True,
            name="batchnorm21_1_25_")
        relu21_1_25_ = mx.symbol.Activation(data=batchnorm21_1_25_,
            act_type='relu',
            name="relu21_1_25_")

        conv22_1_25_ = mx.symbol.pad(data=relu21_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_25_ = mx.symbol.Convolution(data=conv22_1_25_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_25_")
        # conv22_1_25_, output shape: {[8,28,28]}

        batchnorm22_1_25_ = mx.symbol.BatchNorm(data=conv22_1_25_,
            fix_gamma=True,
            name="batchnorm22_1_25_")
        relu22_1_25_ = mx.symbol.Activation(data=batchnorm22_1_25_,
            act_type='relu',
            name="relu22_1_25_")

        conv23_1_25_ = mx.symbol.Convolution(data=relu22_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_25_")
        # conv23_1_25_, output shape: {[512,28,28]}

        batchnorm23_1_25_ = mx.symbol.BatchNorm(data=conv23_1_25_,
            fix_gamma=True,
            name="batchnorm23_1_25_")
        conv21_1_26_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_26_")
        # conv21_1_26_, output shape: {[8,56,56]}

        batchnorm21_1_26_ = mx.symbol.BatchNorm(data=conv21_1_26_,
            fix_gamma=True,
            name="batchnorm21_1_26_")
        relu21_1_26_ = mx.symbol.Activation(data=batchnorm21_1_26_,
            act_type='relu',
            name="relu21_1_26_")

        conv22_1_26_ = mx.symbol.pad(data=relu21_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_26_ = mx.symbol.Convolution(data=conv22_1_26_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_26_")
        # conv22_1_26_, output shape: {[8,28,28]}

        batchnorm22_1_26_ = mx.symbol.BatchNorm(data=conv22_1_26_,
            fix_gamma=True,
            name="batchnorm22_1_26_")
        relu22_1_26_ = mx.symbol.Activation(data=batchnorm22_1_26_,
            act_type='relu',
            name="relu22_1_26_")

        conv23_1_26_ = mx.symbol.Convolution(data=relu22_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_26_")
        # conv23_1_26_, output shape: {[512,28,28]}

        batchnorm23_1_26_ = mx.symbol.BatchNorm(data=conv23_1_26_,
            fix_gamma=True,
            name="batchnorm23_1_26_")
        conv21_1_27_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_27_")
        # conv21_1_27_, output shape: {[8,56,56]}

        batchnorm21_1_27_ = mx.symbol.BatchNorm(data=conv21_1_27_,
            fix_gamma=True,
            name="batchnorm21_1_27_")
        relu21_1_27_ = mx.symbol.Activation(data=batchnorm21_1_27_,
            act_type='relu',
            name="relu21_1_27_")

        conv22_1_27_ = mx.symbol.pad(data=relu21_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_27_ = mx.symbol.Convolution(data=conv22_1_27_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_27_")
        # conv22_1_27_, output shape: {[8,28,28]}

        batchnorm22_1_27_ = mx.symbol.BatchNorm(data=conv22_1_27_,
            fix_gamma=True,
            name="batchnorm22_1_27_")
        relu22_1_27_ = mx.symbol.Activation(data=batchnorm22_1_27_,
            act_type='relu',
            name="relu22_1_27_")

        conv23_1_27_ = mx.symbol.Convolution(data=relu22_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_27_")
        # conv23_1_27_, output shape: {[512,28,28]}

        batchnorm23_1_27_ = mx.symbol.BatchNorm(data=conv23_1_27_,
            fix_gamma=True,
            name="batchnorm23_1_27_")
        conv21_1_28_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_28_")
        # conv21_1_28_, output shape: {[8,56,56]}

        batchnorm21_1_28_ = mx.symbol.BatchNorm(data=conv21_1_28_,
            fix_gamma=True,
            name="batchnorm21_1_28_")
        relu21_1_28_ = mx.symbol.Activation(data=batchnorm21_1_28_,
            act_type='relu',
            name="relu21_1_28_")

        conv22_1_28_ = mx.symbol.pad(data=relu21_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_28_ = mx.symbol.Convolution(data=conv22_1_28_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_28_")
        # conv22_1_28_, output shape: {[8,28,28]}

        batchnorm22_1_28_ = mx.symbol.BatchNorm(data=conv22_1_28_,
            fix_gamma=True,
            name="batchnorm22_1_28_")
        relu22_1_28_ = mx.symbol.Activation(data=batchnorm22_1_28_,
            act_type='relu',
            name="relu22_1_28_")

        conv23_1_28_ = mx.symbol.Convolution(data=relu22_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_28_")
        # conv23_1_28_, output shape: {[512,28,28]}

        batchnorm23_1_28_ = mx.symbol.BatchNorm(data=conv23_1_28_,
            fix_gamma=True,
            name="batchnorm23_1_28_")
        conv21_1_29_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_29_")
        # conv21_1_29_, output shape: {[8,56,56]}

        batchnorm21_1_29_ = mx.symbol.BatchNorm(data=conv21_1_29_,
            fix_gamma=True,
            name="batchnorm21_1_29_")
        relu21_1_29_ = mx.symbol.Activation(data=batchnorm21_1_29_,
            act_type='relu',
            name="relu21_1_29_")

        conv22_1_29_ = mx.symbol.pad(data=relu21_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_29_ = mx.symbol.Convolution(data=conv22_1_29_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_29_")
        # conv22_1_29_, output shape: {[8,28,28]}

        batchnorm22_1_29_ = mx.symbol.BatchNorm(data=conv22_1_29_,
            fix_gamma=True,
            name="batchnorm22_1_29_")
        relu22_1_29_ = mx.symbol.Activation(data=batchnorm22_1_29_,
            act_type='relu',
            name="relu22_1_29_")

        conv23_1_29_ = mx.symbol.Convolution(data=relu22_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_29_")
        # conv23_1_29_, output shape: {[512,28,28]}

        batchnorm23_1_29_ = mx.symbol.BatchNorm(data=conv23_1_29_,
            fix_gamma=True,
            name="batchnorm23_1_29_")
        conv21_1_30_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_30_")
        # conv21_1_30_, output shape: {[8,56,56]}

        batchnorm21_1_30_ = mx.symbol.BatchNorm(data=conv21_1_30_,
            fix_gamma=True,
            name="batchnorm21_1_30_")
        relu21_1_30_ = mx.symbol.Activation(data=batchnorm21_1_30_,
            act_type='relu',
            name="relu21_1_30_")

        conv22_1_30_ = mx.symbol.pad(data=relu21_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_30_ = mx.symbol.Convolution(data=conv22_1_30_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_30_")
        # conv22_1_30_, output shape: {[8,28,28]}

        batchnorm22_1_30_ = mx.symbol.BatchNorm(data=conv22_1_30_,
            fix_gamma=True,
            name="batchnorm22_1_30_")
        relu22_1_30_ = mx.symbol.Activation(data=batchnorm22_1_30_,
            act_type='relu',
            name="relu22_1_30_")

        conv23_1_30_ = mx.symbol.Convolution(data=relu22_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_30_")
        # conv23_1_30_, output shape: {[512,28,28]}

        batchnorm23_1_30_ = mx.symbol.BatchNorm(data=conv23_1_30_,
            fix_gamma=True,
            name="batchnorm23_1_30_")
        conv21_1_31_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_31_")
        # conv21_1_31_, output shape: {[8,56,56]}

        batchnorm21_1_31_ = mx.symbol.BatchNorm(data=conv21_1_31_,
            fix_gamma=True,
            name="batchnorm21_1_31_")
        relu21_1_31_ = mx.symbol.Activation(data=batchnorm21_1_31_,
            act_type='relu',
            name="relu21_1_31_")

        conv22_1_31_ = mx.symbol.pad(data=relu21_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_31_ = mx.symbol.Convolution(data=conv22_1_31_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_31_")
        # conv22_1_31_, output shape: {[8,28,28]}

        batchnorm22_1_31_ = mx.symbol.BatchNorm(data=conv22_1_31_,
            fix_gamma=True,
            name="batchnorm22_1_31_")
        relu22_1_31_ = mx.symbol.Activation(data=batchnorm22_1_31_,
            act_type='relu',
            name="relu22_1_31_")

        conv23_1_31_ = mx.symbol.Convolution(data=relu22_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_31_")
        # conv23_1_31_, output shape: {[512,28,28]}

        batchnorm23_1_31_ = mx.symbol.BatchNorm(data=conv23_1_31_,
            fix_gamma=True,
            name="batchnorm23_1_31_")
        conv21_1_32_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv21_1_32_")
        # conv21_1_32_, output shape: {[8,56,56]}

        batchnorm21_1_32_ = mx.symbol.BatchNorm(data=conv21_1_32_,
            fix_gamma=True,
            name="batchnorm21_1_32_")
        relu21_1_32_ = mx.symbol.Activation(data=batchnorm21_1_32_,
            act_type='relu',
            name="relu21_1_32_")

        conv22_1_32_ = mx.symbol.pad(data=relu21_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv22_1_32_ = mx.symbol.Convolution(data=conv22_1_32_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=8,
            no_bias=False,
            name="conv22_1_32_")
        # conv22_1_32_, output shape: {[8,28,28]}

        batchnorm22_1_32_ = mx.symbol.BatchNorm(data=conv22_1_32_,
            fix_gamma=True,
            name="batchnorm22_1_32_")
        relu22_1_32_ = mx.symbol.Activation(data=batchnorm22_1_32_,
            act_type='relu',
            name="relu22_1_32_")

        conv23_1_32_ = mx.symbol.Convolution(data=relu22_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv23_1_32_")
        # conv23_1_32_, output shape: {[512,28,28]}

        batchnorm23_1_32_ = mx.symbol.BatchNorm(data=conv23_1_32_,
            fix_gamma=True,
            name="batchnorm23_1_32_")
        add24_1_ = batchnorm23_1_1_ + batchnorm23_1_2_ + batchnorm23_1_3_ + batchnorm23_1_4_ + batchnorm23_1_5_ + batchnorm23_1_6_ + batchnorm23_1_7_ + batchnorm23_1_8_ + batchnorm23_1_9_ + batchnorm23_1_10_ + batchnorm23_1_11_ + batchnorm23_1_12_ + batchnorm23_1_13_ + batchnorm23_1_14_ + batchnorm23_1_15_ + batchnorm23_1_16_ + batchnorm23_1_17_ + batchnorm23_1_18_ + batchnorm23_1_19_ + batchnorm23_1_20_ + batchnorm23_1_21_ + batchnorm23_1_22_ + batchnorm23_1_23_ + batchnorm23_1_24_ + batchnorm23_1_25_ + batchnorm23_1_26_ + batchnorm23_1_27_ + batchnorm23_1_28_ + batchnorm23_1_29_ + batchnorm23_1_30_ + batchnorm23_1_31_ + batchnorm23_1_32_
        # add24_1_, output shape: {[512,28,28]}

        conv20_2_ = mx.symbol.Convolution(data=relu19_,
            kernel=(1,1),
            stride=(2,2),
            num_filter=512,
            no_bias=False,
            name="conv20_2_")
        # conv20_2_, output shape: {[512,28,28]}

        batchnorm20_2_ = mx.symbol.BatchNorm(data=conv20_2_,
            fix_gamma=True,
            name="batchnorm20_2_")
        add25_ = add24_1_ + batchnorm20_2_
        # add25_, output shape: {[512,28,28]}

        relu25_ = mx.symbol.Activation(data=add25_,
            act_type='relu',
            name="relu25_")

        conv27_1_1_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_1_")
        # conv27_1_1_, output shape: {[8,28,28]}

        batchnorm27_1_1_ = mx.symbol.BatchNorm(data=conv27_1_1_,
            fix_gamma=True,
            name="batchnorm27_1_1_")
        relu27_1_1_ = mx.symbol.Activation(data=batchnorm27_1_1_,
            act_type='relu',
            name="relu27_1_1_")

        conv28_1_1_ = mx.symbol.pad(data=relu27_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_1_ = mx.symbol.Convolution(data=conv28_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_1_")
        # conv28_1_1_, output shape: {[8,28,28]}

        batchnorm28_1_1_ = mx.symbol.BatchNorm(data=conv28_1_1_,
            fix_gamma=True,
            name="batchnorm28_1_1_")
        relu28_1_1_ = mx.symbol.Activation(data=batchnorm28_1_1_,
            act_type='relu',
            name="relu28_1_1_")

        conv29_1_1_ = mx.symbol.Convolution(data=relu28_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_1_")
        # conv29_1_1_, output shape: {[512,28,28]}

        batchnorm29_1_1_ = mx.symbol.BatchNorm(data=conv29_1_1_,
            fix_gamma=True,
            name="batchnorm29_1_1_")
        conv27_1_2_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_2_")
        # conv27_1_2_, output shape: {[8,28,28]}

        batchnorm27_1_2_ = mx.symbol.BatchNorm(data=conv27_1_2_,
            fix_gamma=True,
            name="batchnorm27_1_2_")
        relu27_1_2_ = mx.symbol.Activation(data=batchnorm27_1_2_,
            act_type='relu',
            name="relu27_1_2_")

        conv28_1_2_ = mx.symbol.pad(data=relu27_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_2_ = mx.symbol.Convolution(data=conv28_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_2_")
        # conv28_1_2_, output shape: {[8,28,28]}

        batchnorm28_1_2_ = mx.symbol.BatchNorm(data=conv28_1_2_,
            fix_gamma=True,
            name="batchnorm28_1_2_")
        relu28_1_2_ = mx.symbol.Activation(data=batchnorm28_1_2_,
            act_type='relu',
            name="relu28_1_2_")

        conv29_1_2_ = mx.symbol.Convolution(data=relu28_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_2_")
        # conv29_1_2_, output shape: {[512,28,28]}

        batchnorm29_1_2_ = mx.symbol.BatchNorm(data=conv29_1_2_,
            fix_gamma=True,
            name="batchnorm29_1_2_")
        conv27_1_3_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_3_")
        # conv27_1_3_, output shape: {[8,28,28]}

        batchnorm27_1_3_ = mx.symbol.BatchNorm(data=conv27_1_3_,
            fix_gamma=True,
            name="batchnorm27_1_3_")
        relu27_1_3_ = mx.symbol.Activation(data=batchnorm27_1_3_,
            act_type='relu',
            name="relu27_1_3_")

        conv28_1_3_ = mx.symbol.pad(data=relu27_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_3_ = mx.symbol.Convolution(data=conv28_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_3_")
        # conv28_1_3_, output shape: {[8,28,28]}

        batchnorm28_1_3_ = mx.symbol.BatchNorm(data=conv28_1_3_,
            fix_gamma=True,
            name="batchnorm28_1_3_")
        relu28_1_3_ = mx.symbol.Activation(data=batchnorm28_1_3_,
            act_type='relu',
            name="relu28_1_3_")

        conv29_1_3_ = mx.symbol.Convolution(data=relu28_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_3_")
        # conv29_1_3_, output shape: {[512,28,28]}

        batchnorm29_1_3_ = mx.symbol.BatchNorm(data=conv29_1_3_,
            fix_gamma=True,
            name="batchnorm29_1_3_")
        conv27_1_4_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_4_")
        # conv27_1_4_, output shape: {[8,28,28]}

        batchnorm27_1_4_ = mx.symbol.BatchNorm(data=conv27_1_4_,
            fix_gamma=True,
            name="batchnorm27_1_4_")
        relu27_1_4_ = mx.symbol.Activation(data=batchnorm27_1_4_,
            act_type='relu',
            name="relu27_1_4_")

        conv28_1_4_ = mx.symbol.pad(data=relu27_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_4_ = mx.symbol.Convolution(data=conv28_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_4_")
        # conv28_1_4_, output shape: {[8,28,28]}

        batchnorm28_1_4_ = mx.symbol.BatchNorm(data=conv28_1_4_,
            fix_gamma=True,
            name="batchnorm28_1_4_")
        relu28_1_4_ = mx.symbol.Activation(data=batchnorm28_1_4_,
            act_type='relu',
            name="relu28_1_4_")

        conv29_1_4_ = mx.symbol.Convolution(data=relu28_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_4_")
        # conv29_1_4_, output shape: {[512,28,28]}

        batchnorm29_1_4_ = mx.symbol.BatchNorm(data=conv29_1_4_,
            fix_gamma=True,
            name="batchnorm29_1_4_")
        conv27_1_5_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_5_")
        # conv27_1_5_, output shape: {[8,28,28]}

        batchnorm27_1_5_ = mx.symbol.BatchNorm(data=conv27_1_5_,
            fix_gamma=True,
            name="batchnorm27_1_5_")
        relu27_1_5_ = mx.symbol.Activation(data=batchnorm27_1_5_,
            act_type='relu',
            name="relu27_1_5_")

        conv28_1_5_ = mx.symbol.pad(data=relu27_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_5_ = mx.symbol.Convolution(data=conv28_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_5_")
        # conv28_1_5_, output shape: {[8,28,28]}

        batchnorm28_1_5_ = mx.symbol.BatchNorm(data=conv28_1_5_,
            fix_gamma=True,
            name="batchnorm28_1_5_")
        relu28_1_5_ = mx.symbol.Activation(data=batchnorm28_1_5_,
            act_type='relu',
            name="relu28_1_5_")

        conv29_1_5_ = mx.symbol.Convolution(data=relu28_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_5_")
        # conv29_1_5_, output shape: {[512,28,28]}

        batchnorm29_1_5_ = mx.symbol.BatchNorm(data=conv29_1_5_,
            fix_gamma=True,
            name="batchnorm29_1_5_")
        conv27_1_6_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_6_")
        # conv27_1_6_, output shape: {[8,28,28]}

        batchnorm27_1_6_ = mx.symbol.BatchNorm(data=conv27_1_6_,
            fix_gamma=True,
            name="batchnorm27_1_6_")
        relu27_1_6_ = mx.symbol.Activation(data=batchnorm27_1_6_,
            act_type='relu',
            name="relu27_1_6_")

        conv28_1_6_ = mx.symbol.pad(data=relu27_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_6_ = mx.symbol.Convolution(data=conv28_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_6_")
        # conv28_1_6_, output shape: {[8,28,28]}

        batchnorm28_1_6_ = mx.symbol.BatchNorm(data=conv28_1_6_,
            fix_gamma=True,
            name="batchnorm28_1_6_")
        relu28_1_6_ = mx.symbol.Activation(data=batchnorm28_1_6_,
            act_type='relu',
            name="relu28_1_6_")

        conv29_1_6_ = mx.symbol.Convolution(data=relu28_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_6_")
        # conv29_1_6_, output shape: {[512,28,28]}

        batchnorm29_1_6_ = mx.symbol.BatchNorm(data=conv29_1_6_,
            fix_gamma=True,
            name="batchnorm29_1_6_")
        conv27_1_7_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_7_")
        # conv27_1_7_, output shape: {[8,28,28]}

        batchnorm27_1_7_ = mx.symbol.BatchNorm(data=conv27_1_7_,
            fix_gamma=True,
            name="batchnorm27_1_7_")
        relu27_1_7_ = mx.symbol.Activation(data=batchnorm27_1_7_,
            act_type='relu',
            name="relu27_1_7_")

        conv28_1_7_ = mx.symbol.pad(data=relu27_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_7_ = mx.symbol.Convolution(data=conv28_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_7_")
        # conv28_1_7_, output shape: {[8,28,28]}

        batchnorm28_1_7_ = mx.symbol.BatchNorm(data=conv28_1_7_,
            fix_gamma=True,
            name="batchnorm28_1_7_")
        relu28_1_7_ = mx.symbol.Activation(data=batchnorm28_1_7_,
            act_type='relu',
            name="relu28_1_7_")

        conv29_1_7_ = mx.symbol.Convolution(data=relu28_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_7_")
        # conv29_1_7_, output shape: {[512,28,28]}

        batchnorm29_1_7_ = mx.symbol.BatchNorm(data=conv29_1_7_,
            fix_gamma=True,
            name="batchnorm29_1_7_")
        conv27_1_8_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_8_")
        # conv27_1_8_, output shape: {[8,28,28]}

        batchnorm27_1_8_ = mx.symbol.BatchNorm(data=conv27_1_8_,
            fix_gamma=True,
            name="batchnorm27_1_8_")
        relu27_1_8_ = mx.symbol.Activation(data=batchnorm27_1_8_,
            act_type='relu',
            name="relu27_1_8_")

        conv28_1_8_ = mx.symbol.pad(data=relu27_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_8_ = mx.symbol.Convolution(data=conv28_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_8_")
        # conv28_1_8_, output shape: {[8,28,28]}

        batchnorm28_1_8_ = mx.symbol.BatchNorm(data=conv28_1_8_,
            fix_gamma=True,
            name="batchnorm28_1_8_")
        relu28_1_8_ = mx.symbol.Activation(data=batchnorm28_1_8_,
            act_type='relu',
            name="relu28_1_8_")

        conv29_1_8_ = mx.symbol.Convolution(data=relu28_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_8_")
        # conv29_1_8_, output shape: {[512,28,28]}

        batchnorm29_1_8_ = mx.symbol.BatchNorm(data=conv29_1_8_,
            fix_gamma=True,
            name="batchnorm29_1_8_")
        conv27_1_9_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_9_")
        # conv27_1_9_, output shape: {[8,28,28]}

        batchnorm27_1_9_ = mx.symbol.BatchNorm(data=conv27_1_9_,
            fix_gamma=True,
            name="batchnorm27_1_9_")
        relu27_1_9_ = mx.symbol.Activation(data=batchnorm27_1_9_,
            act_type='relu',
            name="relu27_1_9_")

        conv28_1_9_ = mx.symbol.pad(data=relu27_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_9_ = mx.symbol.Convolution(data=conv28_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_9_")
        # conv28_1_9_, output shape: {[8,28,28]}

        batchnorm28_1_9_ = mx.symbol.BatchNorm(data=conv28_1_9_,
            fix_gamma=True,
            name="batchnorm28_1_9_")
        relu28_1_9_ = mx.symbol.Activation(data=batchnorm28_1_9_,
            act_type='relu',
            name="relu28_1_9_")

        conv29_1_9_ = mx.symbol.Convolution(data=relu28_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_9_")
        # conv29_1_9_, output shape: {[512,28,28]}

        batchnorm29_1_9_ = mx.symbol.BatchNorm(data=conv29_1_9_,
            fix_gamma=True,
            name="batchnorm29_1_9_")
        conv27_1_10_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_10_")
        # conv27_1_10_, output shape: {[8,28,28]}

        batchnorm27_1_10_ = mx.symbol.BatchNorm(data=conv27_1_10_,
            fix_gamma=True,
            name="batchnorm27_1_10_")
        relu27_1_10_ = mx.symbol.Activation(data=batchnorm27_1_10_,
            act_type='relu',
            name="relu27_1_10_")

        conv28_1_10_ = mx.symbol.pad(data=relu27_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_10_ = mx.symbol.Convolution(data=conv28_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_10_")
        # conv28_1_10_, output shape: {[8,28,28]}

        batchnorm28_1_10_ = mx.symbol.BatchNorm(data=conv28_1_10_,
            fix_gamma=True,
            name="batchnorm28_1_10_")
        relu28_1_10_ = mx.symbol.Activation(data=batchnorm28_1_10_,
            act_type='relu',
            name="relu28_1_10_")

        conv29_1_10_ = mx.symbol.Convolution(data=relu28_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_10_")
        # conv29_1_10_, output shape: {[512,28,28]}

        batchnorm29_1_10_ = mx.symbol.BatchNorm(data=conv29_1_10_,
            fix_gamma=True,
            name="batchnorm29_1_10_")
        conv27_1_11_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_11_")
        # conv27_1_11_, output shape: {[8,28,28]}

        batchnorm27_1_11_ = mx.symbol.BatchNorm(data=conv27_1_11_,
            fix_gamma=True,
            name="batchnorm27_1_11_")
        relu27_1_11_ = mx.symbol.Activation(data=batchnorm27_1_11_,
            act_type='relu',
            name="relu27_1_11_")

        conv28_1_11_ = mx.symbol.pad(data=relu27_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_11_ = mx.symbol.Convolution(data=conv28_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_11_")
        # conv28_1_11_, output shape: {[8,28,28]}

        batchnorm28_1_11_ = mx.symbol.BatchNorm(data=conv28_1_11_,
            fix_gamma=True,
            name="batchnorm28_1_11_")
        relu28_1_11_ = mx.symbol.Activation(data=batchnorm28_1_11_,
            act_type='relu',
            name="relu28_1_11_")

        conv29_1_11_ = mx.symbol.Convolution(data=relu28_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_11_")
        # conv29_1_11_, output shape: {[512,28,28]}

        batchnorm29_1_11_ = mx.symbol.BatchNorm(data=conv29_1_11_,
            fix_gamma=True,
            name="batchnorm29_1_11_")
        conv27_1_12_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_12_")
        # conv27_1_12_, output shape: {[8,28,28]}

        batchnorm27_1_12_ = mx.symbol.BatchNorm(data=conv27_1_12_,
            fix_gamma=True,
            name="batchnorm27_1_12_")
        relu27_1_12_ = mx.symbol.Activation(data=batchnorm27_1_12_,
            act_type='relu',
            name="relu27_1_12_")

        conv28_1_12_ = mx.symbol.pad(data=relu27_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_12_ = mx.symbol.Convolution(data=conv28_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_12_")
        # conv28_1_12_, output shape: {[8,28,28]}

        batchnorm28_1_12_ = mx.symbol.BatchNorm(data=conv28_1_12_,
            fix_gamma=True,
            name="batchnorm28_1_12_")
        relu28_1_12_ = mx.symbol.Activation(data=batchnorm28_1_12_,
            act_type='relu',
            name="relu28_1_12_")

        conv29_1_12_ = mx.symbol.Convolution(data=relu28_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_12_")
        # conv29_1_12_, output shape: {[512,28,28]}

        batchnorm29_1_12_ = mx.symbol.BatchNorm(data=conv29_1_12_,
            fix_gamma=True,
            name="batchnorm29_1_12_")
        conv27_1_13_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_13_")
        # conv27_1_13_, output shape: {[8,28,28]}

        batchnorm27_1_13_ = mx.symbol.BatchNorm(data=conv27_1_13_,
            fix_gamma=True,
            name="batchnorm27_1_13_")
        relu27_1_13_ = mx.symbol.Activation(data=batchnorm27_1_13_,
            act_type='relu',
            name="relu27_1_13_")

        conv28_1_13_ = mx.symbol.pad(data=relu27_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_13_ = mx.symbol.Convolution(data=conv28_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_13_")
        # conv28_1_13_, output shape: {[8,28,28]}

        batchnorm28_1_13_ = mx.symbol.BatchNorm(data=conv28_1_13_,
            fix_gamma=True,
            name="batchnorm28_1_13_")
        relu28_1_13_ = mx.symbol.Activation(data=batchnorm28_1_13_,
            act_type='relu',
            name="relu28_1_13_")

        conv29_1_13_ = mx.symbol.Convolution(data=relu28_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_13_")
        # conv29_1_13_, output shape: {[512,28,28]}

        batchnorm29_1_13_ = mx.symbol.BatchNorm(data=conv29_1_13_,
            fix_gamma=True,
            name="batchnorm29_1_13_")
        conv27_1_14_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_14_")
        # conv27_1_14_, output shape: {[8,28,28]}

        batchnorm27_1_14_ = mx.symbol.BatchNorm(data=conv27_1_14_,
            fix_gamma=True,
            name="batchnorm27_1_14_")
        relu27_1_14_ = mx.symbol.Activation(data=batchnorm27_1_14_,
            act_type='relu',
            name="relu27_1_14_")

        conv28_1_14_ = mx.symbol.pad(data=relu27_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_14_ = mx.symbol.Convolution(data=conv28_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_14_")
        # conv28_1_14_, output shape: {[8,28,28]}

        batchnorm28_1_14_ = mx.symbol.BatchNorm(data=conv28_1_14_,
            fix_gamma=True,
            name="batchnorm28_1_14_")
        relu28_1_14_ = mx.symbol.Activation(data=batchnorm28_1_14_,
            act_type='relu',
            name="relu28_1_14_")

        conv29_1_14_ = mx.symbol.Convolution(data=relu28_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_14_")
        # conv29_1_14_, output shape: {[512,28,28]}

        batchnorm29_1_14_ = mx.symbol.BatchNorm(data=conv29_1_14_,
            fix_gamma=True,
            name="batchnorm29_1_14_")
        conv27_1_15_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_15_")
        # conv27_1_15_, output shape: {[8,28,28]}

        batchnorm27_1_15_ = mx.symbol.BatchNorm(data=conv27_1_15_,
            fix_gamma=True,
            name="batchnorm27_1_15_")
        relu27_1_15_ = mx.symbol.Activation(data=batchnorm27_1_15_,
            act_type='relu',
            name="relu27_1_15_")

        conv28_1_15_ = mx.symbol.pad(data=relu27_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_15_ = mx.symbol.Convolution(data=conv28_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_15_")
        # conv28_1_15_, output shape: {[8,28,28]}

        batchnorm28_1_15_ = mx.symbol.BatchNorm(data=conv28_1_15_,
            fix_gamma=True,
            name="batchnorm28_1_15_")
        relu28_1_15_ = mx.symbol.Activation(data=batchnorm28_1_15_,
            act_type='relu',
            name="relu28_1_15_")

        conv29_1_15_ = mx.symbol.Convolution(data=relu28_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_15_")
        # conv29_1_15_, output shape: {[512,28,28]}

        batchnorm29_1_15_ = mx.symbol.BatchNorm(data=conv29_1_15_,
            fix_gamma=True,
            name="batchnorm29_1_15_")
        conv27_1_16_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_16_")
        # conv27_1_16_, output shape: {[8,28,28]}

        batchnorm27_1_16_ = mx.symbol.BatchNorm(data=conv27_1_16_,
            fix_gamma=True,
            name="batchnorm27_1_16_")
        relu27_1_16_ = mx.symbol.Activation(data=batchnorm27_1_16_,
            act_type='relu',
            name="relu27_1_16_")

        conv28_1_16_ = mx.symbol.pad(data=relu27_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_16_ = mx.symbol.Convolution(data=conv28_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_16_")
        # conv28_1_16_, output shape: {[8,28,28]}

        batchnorm28_1_16_ = mx.symbol.BatchNorm(data=conv28_1_16_,
            fix_gamma=True,
            name="batchnorm28_1_16_")
        relu28_1_16_ = mx.symbol.Activation(data=batchnorm28_1_16_,
            act_type='relu',
            name="relu28_1_16_")

        conv29_1_16_ = mx.symbol.Convolution(data=relu28_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_16_")
        # conv29_1_16_, output shape: {[512,28,28]}

        batchnorm29_1_16_ = mx.symbol.BatchNorm(data=conv29_1_16_,
            fix_gamma=True,
            name="batchnorm29_1_16_")
        conv27_1_17_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_17_")
        # conv27_1_17_, output shape: {[8,28,28]}

        batchnorm27_1_17_ = mx.symbol.BatchNorm(data=conv27_1_17_,
            fix_gamma=True,
            name="batchnorm27_1_17_")
        relu27_1_17_ = mx.symbol.Activation(data=batchnorm27_1_17_,
            act_type='relu',
            name="relu27_1_17_")

        conv28_1_17_ = mx.symbol.pad(data=relu27_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_17_ = mx.symbol.Convolution(data=conv28_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_17_")
        # conv28_1_17_, output shape: {[8,28,28]}

        batchnorm28_1_17_ = mx.symbol.BatchNorm(data=conv28_1_17_,
            fix_gamma=True,
            name="batchnorm28_1_17_")
        relu28_1_17_ = mx.symbol.Activation(data=batchnorm28_1_17_,
            act_type='relu',
            name="relu28_1_17_")

        conv29_1_17_ = mx.symbol.Convolution(data=relu28_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_17_")
        # conv29_1_17_, output shape: {[512,28,28]}

        batchnorm29_1_17_ = mx.symbol.BatchNorm(data=conv29_1_17_,
            fix_gamma=True,
            name="batchnorm29_1_17_")
        conv27_1_18_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_18_")
        # conv27_1_18_, output shape: {[8,28,28]}

        batchnorm27_1_18_ = mx.symbol.BatchNorm(data=conv27_1_18_,
            fix_gamma=True,
            name="batchnorm27_1_18_")
        relu27_1_18_ = mx.symbol.Activation(data=batchnorm27_1_18_,
            act_type='relu',
            name="relu27_1_18_")

        conv28_1_18_ = mx.symbol.pad(data=relu27_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_18_ = mx.symbol.Convolution(data=conv28_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_18_")
        # conv28_1_18_, output shape: {[8,28,28]}

        batchnorm28_1_18_ = mx.symbol.BatchNorm(data=conv28_1_18_,
            fix_gamma=True,
            name="batchnorm28_1_18_")
        relu28_1_18_ = mx.symbol.Activation(data=batchnorm28_1_18_,
            act_type='relu',
            name="relu28_1_18_")

        conv29_1_18_ = mx.symbol.Convolution(data=relu28_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_18_")
        # conv29_1_18_, output shape: {[512,28,28]}

        batchnorm29_1_18_ = mx.symbol.BatchNorm(data=conv29_1_18_,
            fix_gamma=True,
            name="batchnorm29_1_18_")
        conv27_1_19_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_19_")
        # conv27_1_19_, output shape: {[8,28,28]}

        batchnorm27_1_19_ = mx.symbol.BatchNorm(data=conv27_1_19_,
            fix_gamma=True,
            name="batchnorm27_1_19_")
        relu27_1_19_ = mx.symbol.Activation(data=batchnorm27_1_19_,
            act_type='relu',
            name="relu27_1_19_")

        conv28_1_19_ = mx.symbol.pad(data=relu27_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_19_ = mx.symbol.Convolution(data=conv28_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_19_")
        # conv28_1_19_, output shape: {[8,28,28]}

        batchnorm28_1_19_ = mx.symbol.BatchNorm(data=conv28_1_19_,
            fix_gamma=True,
            name="batchnorm28_1_19_")
        relu28_1_19_ = mx.symbol.Activation(data=batchnorm28_1_19_,
            act_type='relu',
            name="relu28_1_19_")

        conv29_1_19_ = mx.symbol.Convolution(data=relu28_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_19_")
        # conv29_1_19_, output shape: {[512,28,28]}

        batchnorm29_1_19_ = mx.symbol.BatchNorm(data=conv29_1_19_,
            fix_gamma=True,
            name="batchnorm29_1_19_")
        conv27_1_20_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_20_")
        # conv27_1_20_, output shape: {[8,28,28]}

        batchnorm27_1_20_ = mx.symbol.BatchNorm(data=conv27_1_20_,
            fix_gamma=True,
            name="batchnorm27_1_20_")
        relu27_1_20_ = mx.symbol.Activation(data=batchnorm27_1_20_,
            act_type='relu',
            name="relu27_1_20_")

        conv28_1_20_ = mx.symbol.pad(data=relu27_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_20_ = mx.symbol.Convolution(data=conv28_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_20_")
        # conv28_1_20_, output shape: {[8,28,28]}

        batchnorm28_1_20_ = mx.symbol.BatchNorm(data=conv28_1_20_,
            fix_gamma=True,
            name="batchnorm28_1_20_")
        relu28_1_20_ = mx.symbol.Activation(data=batchnorm28_1_20_,
            act_type='relu',
            name="relu28_1_20_")

        conv29_1_20_ = mx.symbol.Convolution(data=relu28_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_20_")
        # conv29_1_20_, output shape: {[512,28,28]}

        batchnorm29_1_20_ = mx.symbol.BatchNorm(data=conv29_1_20_,
            fix_gamma=True,
            name="batchnorm29_1_20_")
        conv27_1_21_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_21_")
        # conv27_1_21_, output shape: {[8,28,28]}

        batchnorm27_1_21_ = mx.symbol.BatchNorm(data=conv27_1_21_,
            fix_gamma=True,
            name="batchnorm27_1_21_")
        relu27_1_21_ = mx.symbol.Activation(data=batchnorm27_1_21_,
            act_type='relu',
            name="relu27_1_21_")

        conv28_1_21_ = mx.symbol.pad(data=relu27_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_21_ = mx.symbol.Convolution(data=conv28_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_21_")
        # conv28_1_21_, output shape: {[8,28,28]}

        batchnorm28_1_21_ = mx.symbol.BatchNorm(data=conv28_1_21_,
            fix_gamma=True,
            name="batchnorm28_1_21_")
        relu28_1_21_ = mx.symbol.Activation(data=batchnorm28_1_21_,
            act_type='relu',
            name="relu28_1_21_")

        conv29_1_21_ = mx.symbol.Convolution(data=relu28_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_21_")
        # conv29_1_21_, output shape: {[512,28,28]}

        batchnorm29_1_21_ = mx.symbol.BatchNorm(data=conv29_1_21_,
            fix_gamma=True,
            name="batchnorm29_1_21_")
        conv27_1_22_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_22_")
        # conv27_1_22_, output shape: {[8,28,28]}

        batchnorm27_1_22_ = mx.symbol.BatchNorm(data=conv27_1_22_,
            fix_gamma=True,
            name="batchnorm27_1_22_")
        relu27_1_22_ = mx.symbol.Activation(data=batchnorm27_1_22_,
            act_type='relu',
            name="relu27_1_22_")

        conv28_1_22_ = mx.symbol.pad(data=relu27_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_22_ = mx.symbol.Convolution(data=conv28_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_22_")
        # conv28_1_22_, output shape: {[8,28,28]}

        batchnorm28_1_22_ = mx.symbol.BatchNorm(data=conv28_1_22_,
            fix_gamma=True,
            name="batchnorm28_1_22_")
        relu28_1_22_ = mx.symbol.Activation(data=batchnorm28_1_22_,
            act_type='relu',
            name="relu28_1_22_")

        conv29_1_22_ = mx.symbol.Convolution(data=relu28_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_22_")
        # conv29_1_22_, output shape: {[512,28,28]}

        batchnorm29_1_22_ = mx.symbol.BatchNorm(data=conv29_1_22_,
            fix_gamma=True,
            name="batchnorm29_1_22_")
        conv27_1_23_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_23_")
        # conv27_1_23_, output shape: {[8,28,28]}

        batchnorm27_1_23_ = mx.symbol.BatchNorm(data=conv27_1_23_,
            fix_gamma=True,
            name="batchnorm27_1_23_")
        relu27_1_23_ = mx.symbol.Activation(data=batchnorm27_1_23_,
            act_type='relu',
            name="relu27_1_23_")

        conv28_1_23_ = mx.symbol.pad(data=relu27_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_23_ = mx.symbol.Convolution(data=conv28_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_23_")
        # conv28_1_23_, output shape: {[8,28,28]}

        batchnorm28_1_23_ = mx.symbol.BatchNorm(data=conv28_1_23_,
            fix_gamma=True,
            name="batchnorm28_1_23_")
        relu28_1_23_ = mx.symbol.Activation(data=batchnorm28_1_23_,
            act_type='relu',
            name="relu28_1_23_")

        conv29_1_23_ = mx.symbol.Convolution(data=relu28_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_23_")
        # conv29_1_23_, output shape: {[512,28,28]}

        batchnorm29_1_23_ = mx.symbol.BatchNorm(data=conv29_1_23_,
            fix_gamma=True,
            name="batchnorm29_1_23_")
        conv27_1_24_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_24_")
        # conv27_1_24_, output shape: {[8,28,28]}

        batchnorm27_1_24_ = mx.symbol.BatchNorm(data=conv27_1_24_,
            fix_gamma=True,
            name="batchnorm27_1_24_")
        relu27_1_24_ = mx.symbol.Activation(data=batchnorm27_1_24_,
            act_type='relu',
            name="relu27_1_24_")

        conv28_1_24_ = mx.symbol.pad(data=relu27_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_24_ = mx.symbol.Convolution(data=conv28_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_24_")
        # conv28_1_24_, output shape: {[8,28,28]}

        batchnorm28_1_24_ = mx.symbol.BatchNorm(data=conv28_1_24_,
            fix_gamma=True,
            name="batchnorm28_1_24_")
        relu28_1_24_ = mx.symbol.Activation(data=batchnorm28_1_24_,
            act_type='relu',
            name="relu28_1_24_")

        conv29_1_24_ = mx.symbol.Convolution(data=relu28_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_24_")
        # conv29_1_24_, output shape: {[512,28,28]}

        batchnorm29_1_24_ = mx.symbol.BatchNorm(data=conv29_1_24_,
            fix_gamma=True,
            name="batchnorm29_1_24_")
        conv27_1_25_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_25_")
        # conv27_1_25_, output shape: {[8,28,28]}

        batchnorm27_1_25_ = mx.symbol.BatchNorm(data=conv27_1_25_,
            fix_gamma=True,
            name="batchnorm27_1_25_")
        relu27_1_25_ = mx.symbol.Activation(data=batchnorm27_1_25_,
            act_type='relu',
            name="relu27_1_25_")

        conv28_1_25_ = mx.symbol.pad(data=relu27_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_25_ = mx.symbol.Convolution(data=conv28_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_25_")
        # conv28_1_25_, output shape: {[8,28,28]}

        batchnorm28_1_25_ = mx.symbol.BatchNorm(data=conv28_1_25_,
            fix_gamma=True,
            name="batchnorm28_1_25_")
        relu28_1_25_ = mx.symbol.Activation(data=batchnorm28_1_25_,
            act_type='relu',
            name="relu28_1_25_")

        conv29_1_25_ = mx.symbol.Convolution(data=relu28_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_25_")
        # conv29_1_25_, output shape: {[512,28,28]}

        batchnorm29_1_25_ = mx.symbol.BatchNorm(data=conv29_1_25_,
            fix_gamma=True,
            name="batchnorm29_1_25_")
        conv27_1_26_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_26_")
        # conv27_1_26_, output shape: {[8,28,28]}

        batchnorm27_1_26_ = mx.symbol.BatchNorm(data=conv27_1_26_,
            fix_gamma=True,
            name="batchnorm27_1_26_")
        relu27_1_26_ = mx.symbol.Activation(data=batchnorm27_1_26_,
            act_type='relu',
            name="relu27_1_26_")

        conv28_1_26_ = mx.symbol.pad(data=relu27_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_26_ = mx.symbol.Convolution(data=conv28_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_26_")
        # conv28_1_26_, output shape: {[8,28,28]}

        batchnorm28_1_26_ = mx.symbol.BatchNorm(data=conv28_1_26_,
            fix_gamma=True,
            name="batchnorm28_1_26_")
        relu28_1_26_ = mx.symbol.Activation(data=batchnorm28_1_26_,
            act_type='relu',
            name="relu28_1_26_")

        conv29_1_26_ = mx.symbol.Convolution(data=relu28_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_26_")
        # conv29_1_26_, output shape: {[512,28,28]}

        batchnorm29_1_26_ = mx.symbol.BatchNorm(data=conv29_1_26_,
            fix_gamma=True,
            name="batchnorm29_1_26_")
        conv27_1_27_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_27_")
        # conv27_1_27_, output shape: {[8,28,28]}

        batchnorm27_1_27_ = mx.symbol.BatchNorm(data=conv27_1_27_,
            fix_gamma=True,
            name="batchnorm27_1_27_")
        relu27_1_27_ = mx.symbol.Activation(data=batchnorm27_1_27_,
            act_type='relu',
            name="relu27_1_27_")

        conv28_1_27_ = mx.symbol.pad(data=relu27_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_27_ = mx.symbol.Convolution(data=conv28_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_27_")
        # conv28_1_27_, output shape: {[8,28,28]}

        batchnorm28_1_27_ = mx.symbol.BatchNorm(data=conv28_1_27_,
            fix_gamma=True,
            name="batchnorm28_1_27_")
        relu28_1_27_ = mx.symbol.Activation(data=batchnorm28_1_27_,
            act_type='relu',
            name="relu28_1_27_")

        conv29_1_27_ = mx.symbol.Convolution(data=relu28_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_27_")
        # conv29_1_27_, output shape: {[512,28,28]}

        batchnorm29_1_27_ = mx.symbol.BatchNorm(data=conv29_1_27_,
            fix_gamma=True,
            name="batchnorm29_1_27_")
        conv27_1_28_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_28_")
        # conv27_1_28_, output shape: {[8,28,28]}

        batchnorm27_1_28_ = mx.symbol.BatchNorm(data=conv27_1_28_,
            fix_gamma=True,
            name="batchnorm27_1_28_")
        relu27_1_28_ = mx.symbol.Activation(data=batchnorm27_1_28_,
            act_type='relu',
            name="relu27_1_28_")

        conv28_1_28_ = mx.symbol.pad(data=relu27_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_28_ = mx.symbol.Convolution(data=conv28_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_28_")
        # conv28_1_28_, output shape: {[8,28,28]}

        batchnorm28_1_28_ = mx.symbol.BatchNorm(data=conv28_1_28_,
            fix_gamma=True,
            name="batchnorm28_1_28_")
        relu28_1_28_ = mx.symbol.Activation(data=batchnorm28_1_28_,
            act_type='relu',
            name="relu28_1_28_")

        conv29_1_28_ = mx.symbol.Convolution(data=relu28_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_28_")
        # conv29_1_28_, output shape: {[512,28,28]}

        batchnorm29_1_28_ = mx.symbol.BatchNorm(data=conv29_1_28_,
            fix_gamma=True,
            name="batchnorm29_1_28_")
        conv27_1_29_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_29_")
        # conv27_1_29_, output shape: {[8,28,28]}

        batchnorm27_1_29_ = mx.symbol.BatchNorm(data=conv27_1_29_,
            fix_gamma=True,
            name="batchnorm27_1_29_")
        relu27_1_29_ = mx.symbol.Activation(data=batchnorm27_1_29_,
            act_type='relu',
            name="relu27_1_29_")

        conv28_1_29_ = mx.symbol.pad(data=relu27_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_29_ = mx.symbol.Convolution(data=conv28_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_29_")
        # conv28_1_29_, output shape: {[8,28,28]}

        batchnorm28_1_29_ = mx.symbol.BatchNorm(data=conv28_1_29_,
            fix_gamma=True,
            name="batchnorm28_1_29_")
        relu28_1_29_ = mx.symbol.Activation(data=batchnorm28_1_29_,
            act_type='relu',
            name="relu28_1_29_")

        conv29_1_29_ = mx.symbol.Convolution(data=relu28_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_29_")
        # conv29_1_29_, output shape: {[512,28,28]}

        batchnorm29_1_29_ = mx.symbol.BatchNorm(data=conv29_1_29_,
            fix_gamma=True,
            name="batchnorm29_1_29_")
        conv27_1_30_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_30_")
        # conv27_1_30_, output shape: {[8,28,28]}

        batchnorm27_1_30_ = mx.symbol.BatchNorm(data=conv27_1_30_,
            fix_gamma=True,
            name="batchnorm27_1_30_")
        relu27_1_30_ = mx.symbol.Activation(data=batchnorm27_1_30_,
            act_type='relu',
            name="relu27_1_30_")

        conv28_1_30_ = mx.symbol.pad(data=relu27_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_30_ = mx.symbol.Convolution(data=conv28_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_30_")
        # conv28_1_30_, output shape: {[8,28,28]}

        batchnorm28_1_30_ = mx.symbol.BatchNorm(data=conv28_1_30_,
            fix_gamma=True,
            name="batchnorm28_1_30_")
        relu28_1_30_ = mx.symbol.Activation(data=batchnorm28_1_30_,
            act_type='relu',
            name="relu28_1_30_")

        conv29_1_30_ = mx.symbol.Convolution(data=relu28_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_30_")
        # conv29_1_30_, output shape: {[512,28,28]}

        batchnorm29_1_30_ = mx.symbol.BatchNorm(data=conv29_1_30_,
            fix_gamma=True,
            name="batchnorm29_1_30_")
        conv27_1_31_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_31_")
        # conv27_1_31_, output shape: {[8,28,28]}

        batchnorm27_1_31_ = mx.symbol.BatchNorm(data=conv27_1_31_,
            fix_gamma=True,
            name="batchnorm27_1_31_")
        relu27_1_31_ = mx.symbol.Activation(data=batchnorm27_1_31_,
            act_type='relu',
            name="relu27_1_31_")

        conv28_1_31_ = mx.symbol.pad(data=relu27_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_31_ = mx.symbol.Convolution(data=conv28_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_31_")
        # conv28_1_31_, output shape: {[8,28,28]}

        batchnorm28_1_31_ = mx.symbol.BatchNorm(data=conv28_1_31_,
            fix_gamma=True,
            name="batchnorm28_1_31_")
        relu28_1_31_ = mx.symbol.Activation(data=batchnorm28_1_31_,
            act_type='relu',
            name="relu28_1_31_")

        conv29_1_31_ = mx.symbol.Convolution(data=relu28_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_31_")
        # conv29_1_31_, output shape: {[512,28,28]}

        batchnorm29_1_31_ = mx.symbol.BatchNorm(data=conv29_1_31_,
            fix_gamma=True,
            name="batchnorm29_1_31_")
        conv27_1_32_ = mx.symbol.Convolution(data=relu25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv27_1_32_")
        # conv27_1_32_, output shape: {[8,28,28]}

        batchnorm27_1_32_ = mx.symbol.BatchNorm(data=conv27_1_32_,
            fix_gamma=True,
            name="batchnorm27_1_32_")
        relu27_1_32_ = mx.symbol.Activation(data=batchnorm27_1_32_,
            act_type='relu',
            name="relu27_1_32_")

        conv28_1_32_ = mx.symbol.pad(data=relu27_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv28_1_32_ = mx.symbol.Convolution(data=conv28_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv28_1_32_")
        # conv28_1_32_, output shape: {[8,28,28]}

        batchnorm28_1_32_ = mx.symbol.BatchNorm(data=conv28_1_32_,
            fix_gamma=True,
            name="batchnorm28_1_32_")
        relu28_1_32_ = mx.symbol.Activation(data=batchnorm28_1_32_,
            act_type='relu',
            name="relu28_1_32_")

        conv29_1_32_ = mx.symbol.Convolution(data=relu28_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv29_1_32_")
        # conv29_1_32_, output shape: {[512,28,28]}

        batchnorm29_1_32_ = mx.symbol.BatchNorm(data=conv29_1_32_,
            fix_gamma=True,
            name="batchnorm29_1_32_")
        add30_1_ = batchnorm29_1_1_ + batchnorm29_1_2_ + batchnorm29_1_3_ + batchnorm29_1_4_ + batchnorm29_1_5_ + batchnorm29_1_6_ + batchnorm29_1_7_ + batchnorm29_1_8_ + batchnorm29_1_9_ + batchnorm29_1_10_ + batchnorm29_1_11_ + batchnorm29_1_12_ + batchnorm29_1_13_ + batchnorm29_1_14_ + batchnorm29_1_15_ + batchnorm29_1_16_ + batchnorm29_1_17_ + batchnorm29_1_18_ + batchnorm29_1_19_ + batchnorm29_1_20_ + batchnorm29_1_21_ + batchnorm29_1_22_ + batchnorm29_1_23_ + batchnorm29_1_24_ + batchnorm29_1_25_ + batchnorm29_1_26_ + batchnorm29_1_27_ + batchnorm29_1_28_ + batchnorm29_1_29_ + batchnorm29_1_30_ + batchnorm29_1_31_ + batchnorm29_1_32_
        # add30_1_, output shape: {[512,28,28]}

        add31_ = add30_1_ + relu25_
        # add31_, output shape: {[512,28,28]}

        relu31_ = mx.symbol.Activation(data=add31_,
            act_type='relu',
            name="relu31_")

        conv33_1_1_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_1_")
        # conv33_1_1_, output shape: {[8,28,28]}

        batchnorm33_1_1_ = mx.symbol.BatchNorm(data=conv33_1_1_,
            fix_gamma=True,
            name="batchnorm33_1_1_")
        relu33_1_1_ = mx.symbol.Activation(data=batchnorm33_1_1_,
            act_type='relu',
            name="relu33_1_1_")

        conv34_1_1_ = mx.symbol.pad(data=relu33_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_1_ = mx.symbol.Convolution(data=conv34_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_1_")
        # conv34_1_1_, output shape: {[8,28,28]}

        batchnorm34_1_1_ = mx.symbol.BatchNorm(data=conv34_1_1_,
            fix_gamma=True,
            name="batchnorm34_1_1_")
        relu34_1_1_ = mx.symbol.Activation(data=batchnorm34_1_1_,
            act_type='relu',
            name="relu34_1_1_")

        conv35_1_1_ = mx.symbol.Convolution(data=relu34_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_1_")
        # conv35_1_1_, output shape: {[512,28,28]}

        batchnorm35_1_1_ = mx.symbol.BatchNorm(data=conv35_1_1_,
            fix_gamma=True,
            name="batchnorm35_1_1_")
        conv33_1_2_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_2_")
        # conv33_1_2_, output shape: {[8,28,28]}

        batchnorm33_1_2_ = mx.symbol.BatchNorm(data=conv33_1_2_,
            fix_gamma=True,
            name="batchnorm33_1_2_")
        relu33_1_2_ = mx.symbol.Activation(data=batchnorm33_1_2_,
            act_type='relu',
            name="relu33_1_2_")

        conv34_1_2_ = mx.symbol.pad(data=relu33_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_2_ = mx.symbol.Convolution(data=conv34_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_2_")
        # conv34_1_2_, output shape: {[8,28,28]}

        batchnorm34_1_2_ = mx.symbol.BatchNorm(data=conv34_1_2_,
            fix_gamma=True,
            name="batchnorm34_1_2_")
        relu34_1_2_ = mx.symbol.Activation(data=batchnorm34_1_2_,
            act_type='relu',
            name="relu34_1_2_")

        conv35_1_2_ = mx.symbol.Convolution(data=relu34_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_2_")
        # conv35_1_2_, output shape: {[512,28,28]}

        batchnorm35_1_2_ = mx.symbol.BatchNorm(data=conv35_1_2_,
            fix_gamma=True,
            name="batchnorm35_1_2_")
        conv33_1_3_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_3_")
        # conv33_1_3_, output shape: {[8,28,28]}

        batchnorm33_1_3_ = mx.symbol.BatchNorm(data=conv33_1_3_,
            fix_gamma=True,
            name="batchnorm33_1_3_")
        relu33_1_3_ = mx.symbol.Activation(data=batchnorm33_1_3_,
            act_type='relu',
            name="relu33_1_3_")

        conv34_1_3_ = mx.symbol.pad(data=relu33_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_3_ = mx.symbol.Convolution(data=conv34_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_3_")
        # conv34_1_3_, output shape: {[8,28,28]}

        batchnorm34_1_3_ = mx.symbol.BatchNorm(data=conv34_1_3_,
            fix_gamma=True,
            name="batchnorm34_1_3_")
        relu34_1_3_ = mx.symbol.Activation(data=batchnorm34_1_3_,
            act_type='relu',
            name="relu34_1_3_")

        conv35_1_3_ = mx.symbol.Convolution(data=relu34_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_3_")
        # conv35_1_3_, output shape: {[512,28,28]}

        batchnorm35_1_3_ = mx.symbol.BatchNorm(data=conv35_1_3_,
            fix_gamma=True,
            name="batchnorm35_1_3_")
        conv33_1_4_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_4_")
        # conv33_1_4_, output shape: {[8,28,28]}

        batchnorm33_1_4_ = mx.symbol.BatchNorm(data=conv33_1_4_,
            fix_gamma=True,
            name="batchnorm33_1_4_")
        relu33_1_4_ = mx.symbol.Activation(data=batchnorm33_1_4_,
            act_type='relu',
            name="relu33_1_4_")

        conv34_1_4_ = mx.symbol.pad(data=relu33_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_4_ = mx.symbol.Convolution(data=conv34_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_4_")
        # conv34_1_4_, output shape: {[8,28,28]}

        batchnorm34_1_4_ = mx.symbol.BatchNorm(data=conv34_1_4_,
            fix_gamma=True,
            name="batchnorm34_1_4_")
        relu34_1_4_ = mx.symbol.Activation(data=batchnorm34_1_4_,
            act_type='relu',
            name="relu34_1_4_")

        conv35_1_4_ = mx.symbol.Convolution(data=relu34_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_4_")
        # conv35_1_4_, output shape: {[512,28,28]}

        batchnorm35_1_4_ = mx.symbol.BatchNorm(data=conv35_1_4_,
            fix_gamma=True,
            name="batchnorm35_1_4_")
        conv33_1_5_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_5_")
        # conv33_1_5_, output shape: {[8,28,28]}

        batchnorm33_1_5_ = mx.symbol.BatchNorm(data=conv33_1_5_,
            fix_gamma=True,
            name="batchnorm33_1_5_")
        relu33_1_5_ = mx.symbol.Activation(data=batchnorm33_1_5_,
            act_type='relu',
            name="relu33_1_5_")

        conv34_1_5_ = mx.symbol.pad(data=relu33_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_5_ = mx.symbol.Convolution(data=conv34_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_5_")
        # conv34_1_5_, output shape: {[8,28,28]}

        batchnorm34_1_5_ = mx.symbol.BatchNorm(data=conv34_1_5_,
            fix_gamma=True,
            name="batchnorm34_1_5_")
        relu34_1_5_ = mx.symbol.Activation(data=batchnorm34_1_5_,
            act_type='relu',
            name="relu34_1_5_")

        conv35_1_5_ = mx.symbol.Convolution(data=relu34_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_5_")
        # conv35_1_5_, output shape: {[512,28,28]}

        batchnorm35_1_5_ = mx.symbol.BatchNorm(data=conv35_1_5_,
            fix_gamma=True,
            name="batchnorm35_1_5_")
        conv33_1_6_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_6_")
        # conv33_1_6_, output shape: {[8,28,28]}

        batchnorm33_1_6_ = mx.symbol.BatchNorm(data=conv33_1_6_,
            fix_gamma=True,
            name="batchnorm33_1_6_")
        relu33_1_6_ = mx.symbol.Activation(data=batchnorm33_1_6_,
            act_type='relu',
            name="relu33_1_6_")

        conv34_1_6_ = mx.symbol.pad(data=relu33_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_6_ = mx.symbol.Convolution(data=conv34_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_6_")
        # conv34_1_6_, output shape: {[8,28,28]}

        batchnorm34_1_6_ = mx.symbol.BatchNorm(data=conv34_1_6_,
            fix_gamma=True,
            name="batchnorm34_1_6_")
        relu34_1_6_ = mx.symbol.Activation(data=batchnorm34_1_6_,
            act_type='relu',
            name="relu34_1_6_")

        conv35_1_6_ = mx.symbol.Convolution(data=relu34_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_6_")
        # conv35_1_6_, output shape: {[512,28,28]}

        batchnorm35_1_6_ = mx.symbol.BatchNorm(data=conv35_1_6_,
            fix_gamma=True,
            name="batchnorm35_1_6_")
        conv33_1_7_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_7_")
        # conv33_1_7_, output shape: {[8,28,28]}

        batchnorm33_1_7_ = mx.symbol.BatchNorm(data=conv33_1_7_,
            fix_gamma=True,
            name="batchnorm33_1_7_")
        relu33_1_7_ = mx.symbol.Activation(data=batchnorm33_1_7_,
            act_type='relu',
            name="relu33_1_7_")

        conv34_1_7_ = mx.symbol.pad(data=relu33_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_7_ = mx.symbol.Convolution(data=conv34_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_7_")
        # conv34_1_7_, output shape: {[8,28,28]}

        batchnorm34_1_7_ = mx.symbol.BatchNorm(data=conv34_1_7_,
            fix_gamma=True,
            name="batchnorm34_1_7_")
        relu34_1_7_ = mx.symbol.Activation(data=batchnorm34_1_7_,
            act_type='relu',
            name="relu34_1_7_")

        conv35_1_7_ = mx.symbol.Convolution(data=relu34_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_7_")
        # conv35_1_7_, output shape: {[512,28,28]}

        batchnorm35_1_7_ = mx.symbol.BatchNorm(data=conv35_1_7_,
            fix_gamma=True,
            name="batchnorm35_1_7_")
        conv33_1_8_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_8_")
        # conv33_1_8_, output shape: {[8,28,28]}

        batchnorm33_1_8_ = mx.symbol.BatchNorm(data=conv33_1_8_,
            fix_gamma=True,
            name="batchnorm33_1_8_")
        relu33_1_8_ = mx.symbol.Activation(data=batchnorm33_1_8_,
            act_type='relu',
            name="relu33_1_8_")

        conv34_1_8_ = mx.symbol.pad(data=relu33_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_8_ = mx.symbol.Convolution(data=conv34_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_8_")
        # conv34_1_8_, output shape: {[8,28,28]}

        batchnorm34_1_8_ = mx.symbol.BatchNorm(data=conv34_1_8_,
            fix_gamma=True,
            name="batchnorm34_1_8_")
        relu34_1_8_ = mx.symbol.Activation(data=batchnorm34_1_8_,
            act_type='relu',
            name="relu34_1_8_")

        conv35_1_8_ = mx.symbol.Convolution(data=relu34_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_8_")
        # conv35_1_8_, output shape: {[512,28,28]}

        batchnorm35_1_8_ = mx.symbol.BatchNorm(data=conv35_1_8_,
            fix_gamma=True,
            name="batchnorm35_1_8_")
        conv33_1_9_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_9_")
        # conv33_1_9_, output shape: {[8,28,28]}

        batchnorm33_1_9_ = mx.symbol.BatchNorm(data=conv33_1_9_,
            fix_gamma=True,
            name="batchnorm33_1_9_")
        relu33_1_9_ = mx.symbol.Activation(data=batchnorm33_1_9_,
            act_type='relu',
            name="relu33_1_9_")

        conv34_1_9_ = mx.symbol.pad(data=relu33_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_9_ = mx.symbol.Convolution(data=conv34_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_9_")
        # conv34_1_9_, output shape: {[8,28,28]}

        batchnorm34_1_9_ = mx.symbol.BatchNorm(data=conv34_1_9_,
            fix_gamma=True,
            name="batchnorm34_1_9_")
        relu34_1_9_ = mx.symbol.Activation(data=batchnorm34_1_9_,
            act_type='relu',
            name="relu34_1_9_")

        conv35_1_9_ = mx.symbol.Convolution(data=relu34_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_9_")
        # conv35_1_9_, output shape: {[512,28,28]}

        batchnorm35_1_9_ = mx.symbol.BatchNorm(data=conv35_1_9_,
            fix_gamma=True,
            name="batchnorm35_1_9_")
        conv33_1_10_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_10_")
        # conv33_1_10_, output shape: {[8,28,28]}

        batchnorm33_1_10_ = mx.symbol.BatchNorm(data=conv33_1_10_,
            fix_gamma=True,
            name="batchnorm33_1_10_")
        relu33_1_10_ = mx.symbol.Activation(data=batchnorm33_1_10_,
            act_type='relu',
            name="relu33_1_10_")

        conv34_1_10_ = mx.symbol.pad(data=relu33_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_10_ = mx.symbol.Convolution(data=conv34_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_10_")
        # conv34_1_10_, output shape: {[8,28,28]}

        batchnorm34_1_10_ = mx.symbol.BatchNorm(data=conv34_1_10_,
            fix_gamma=True,
            name="batchnorm34_1_10_")
        relu34_1_10_ = mx.symbol.Activation(data=batchnorm34_1_10_,
            act_type='relu',
            name="relu34_1_10_")

        conv35_1_10_ = mx.symbol.Convolution(data=relu34_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_10_")
        # conv35_1_10_, output shape: {[512,28,28]}

        batchnorm35_1_10_ = mx.symbol.BatchNorm(data=conv35_1_10_,
            fix_gamma=True,
            name="batchnorm35_1_10_")
        conv33_1_11_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_11_")
        # conv33_1_11_, output shape: {[8,28,28]}

        batchnorm33_1_11_ = mx.symbol.BatchNorm(data=conv33_1_11_,
            fix_gamma=True,
            name="batchnorm33_1_11_")
        relu33_1_11_ = mx.symbol.Activation(data=batchnorm33_1_11_,
            act_type='relu',
            name="relu33_1_11_")

        conv34_1_11_ = mx.symbol.pad(data=relu33_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_11_ = mx.symbol.Convolution(data=conv34_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_11_")
        # conv34_1_11_, output shape: {[8,28,28]}

        batchnorm34_1_11_ = mx.symbol.BatchNorm(data=conv34_1_11_,
            fix_gamma=True,
            name="batchnorm34_1_11_")
        relu34_1_11_ = mx.symbol.Activation(data=batchnorm34_1_11_,
            act_type='relu',
            name="relu34_1_11_")

        conv35_1_11_ = mx.symbol.Convolution(data=relu34_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_11_")
        # conv35_1_11_, output shape: {[512,28,28]}

        batchnorm35_1_11_ = mx.symbol.BatchNorm(data=conv35_1_11_,
            fix_gamma=True,
            name="batchnorm35_1_11_")
        conv33_1_12_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_12_")
        # conv33_1_12_, output shape: {[8,28,28]}

        batchnorm33_1_12_ = mx.symbol.BatchNorm(data=conv33_1_12_,
            fix_gamma=True,
            name="batchnorm33_1_12_")
        relu33_1_12_ = mx.symbol.Activation(data=batchnorm33_1_12_,
            act_type='relu',
            name="relu33_1_12_")

        conv34_1_12_ = mx.symbol.pad(data=relu33_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_12_ = mx.symbol.Convolution(data=conv34_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_12_")
        # conv34_1_12_, output shape: {[8,28,28]}

        batchnorm34_1_12_ = mx.symbol.BatchNorm(data=conv34_1_12_,
            fix_gamma=True,
            name="batchnorm34_1_12_")
        relu34_1_12_ = mx.symbol.Activation(data=batchnorm34_1_12_,
            act_type='relu',
            name="relu34_1_12_")

        conv35_1_12_ = mx.symbol.Convolution(data=relu34_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_12_")
        # conv35_1_12_, output shape: {[512,28,28]}

        batchnorm35_1_12_ = mx.symbol.BatchNorm(data=conv35_1_12_,
            fix_gamma=True,
            name="batchnorm35_1_12_")
        conv33_1_13_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_13_")
        # conv33_1_13_, output shape: {[8,28,28]}

        batchnorm33_1_13_ = mx.symbol.BatchNorm(data=conv33_1_13_,
            fix_gamma=True,
            name="batchnorm33_1_13_")
        relu33_1_13_ = mx.symbol.Activation(data=batchnorm33_1_13_,
            act_type='relu',
            name="relu33_1_13_")

        conv34_1_13_ = mx.symbol.pad(data=relu33_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_13_ = mx.symbol.Convolution(data=conv34_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_13_")
        # conv34_1_13_, output shape: {[8,28,28]}

        batchnorm34_1_13_ = mx.symbol.BatchNorm(data=conv34_1_13_,
            fix_gamma=True,
            name="batchnorm34_1_13_")
        relu34_1_13_ = mx.symbol.Activation(data=batchnorm34_1_13_,
            act_type='relu',
            name="relu34_1_13_")

        conv35_1_13_ = mx.symbol.Convolution(data=relu34_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_13_")
        # conv35_1_13_, output shape: {[512,28,28]}

        batchnorm35_1_13_ = mx.symbol.BatchNorm(data=conv35_1_13_,
            fix_gamma=True,
            name="batchnorm35_1_13_")
        conv33_1_14_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_14_")
        # conv33_1_14_, output shape: {[8,28,28]}

        batchnorm33_1_14_ = mx.symbol.BatchNorm(data=conv33_1_14_,
            fix_gamma=True,
            name="batchnorm33_1_14_")
        relu33_1_14_ = mx.symbol.Activation(data=batchnorm33_1_14_,
            act_type='relu',
            name="relu33_1_14_")

        conv34_1_14_ = mx.symbol.pad(data=relu33_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_14_ = mx.symbol.Convolution(data=conv34_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_14_")
        # conv34_1_14_, output shape: {[8,28,28]}

        batchnorm34_1_14_ = mx.symbol.BatchNorm(data=conv34_1_14_,
            fix_gamma=True,
            name="batchnorm34_1_14_")
        relu34_1_14_ = mx.symbol.Activation(data=batchnorm34_1_14_,
            act_type='relu',
            name="relu34_1_14_")

        conv35_1_14_ = mx.symbol.Convolution(data=relu34_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_14_")
        # conv35_1_14_, output shape: {[512,28,28]}

        batchnorm35_1_14_ = mx.symbol.BatchNorm(data=conv35_1_14_,
            fix_gamma=True,
            name="batchnorm35_1_14_")
        conv33_1_15_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_15_")
        # conv33_1_15_, output shape: {[8,28,28]}

        batchnorm33_1_15_ = mx.symbol.BatchNorm(data=conv33_1_15_,
            fix_gamma=True,
            name="batchnorm33_1_15_")
        relu33_1_15_ = mx.symbol.Activation(data=batchnorm33_1_15_,
            act_type='relu',
            name="relu33_1_15_")

        conv34_1_15_ = mx.symbol.pad(data=relu33_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_15_ = mx.symbol.Convolution(data=conv34_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_15_")
        # conv34_1_15_, output shape: {[8,28,28]}

        batchnorm34_1_15_ = mx.symbol.BatchNorm(data=conv34_1_15_,
            fix_gamma=True,
            name="batchnorm34_1_15_")
        relu34_1_15_ = mx.symbol.Activation(data=batchnorm34_1_15_,
            act_type='relu',
            name="relu34_1_15_")

        conv35_1_15_ = mx.symbol.Convolution(data=relu34_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_15_")
        # conv35_1_15_, output shape: {[512,28,28]}

        batchnorm35_1_15_ = mx.symbol.BatchNorm(data=conv35_1_15_,
            fix_gamma=True,
            name="batchnorm35_1_15_")
        conv33_1_16_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_16_")
        # conv33_1_16_, output shape: {[8,28,28]}

        batchnorm33_1_16_ = mx.symbol.BatchNorm(data=conv33_1_16_,
            fix_gamma=True,
            name="batchnorm33_1_16_")
        relu33_1_16_ = mx.symbol.Activation(data=batchnorm33_1_16_,
            act_type='relu',
            name="relu33_1_16_")

        conv34_1_16_ = mx.symbol.pad(data=relu33_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_16_ = mx.symbol.Convolution(data=conv34_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_16_")
        # conv34_1_16_, output shape: {[8,28,28]}

        batchnorm34_1_16_ = mx.symbol.BatchNorm(data=conv34_1_16_,
            fix_gamma=True,
            name="batchnorm34_1_16_")
        relu34_1_16_ = mx.symbol.Activation(data=batchnorm34_1_16_,
            act_type='relu',
            name="relu34_1_16_")

        conv35_1_16_ = mx.symbol.Convolution(data=relu34_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_16_")
        # conv35_1_16_, output shape: {[512,28,28]}

        batchnorm35_1_16_ = mx.symbol.BatchNorm(data=conv35_1_16_,
            fix_gamma=True,
            name="batchnorm35_1_16_")
        conv33_1_17_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_17_")
        # conv33_1_17_, output shape: {[8,28,28]}

        batchnorm33_1_17_ = mx.symbol.BatchNorm(data=conv33_1_17_,
            fix_gamma=True,
            name="batchnorm33_1_17_")
        relu33_1_17_ = mx.symbol.Activation(data=batchnorm33_1_17_,
            act_type='relu',
            name="relu33_1_17_")

        conv34_1_17_ = mx.symbol.pad(data=relu33_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_17_ = mx.symbol.Convolution(data=conv34_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_17_")
        # conv34_1_17_, output shape: {[8,28,28]}

        batchnorm34_1_17_ = mx.symbol.BatchNorm(data=conv34_1_17_,
            fix_gamma=True,
            name="batchnorm34_1_17_")
        relu34_1_17_ = mx.symbol.Activation(data=batchnorm34_1_17_,
            act_type='relu',
            name="relu34_1_17_")

        conv35_1_17_ = mx.symbol.Convolution(data=relu34_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_17_")
        # conv35_1_17_, output shape: {[512,28,28]}

        batchnorm35_1_17_ = mx.symbol.BatchNorm(data=conv35_1_17_,
            fix_gamma=True,
            name="batchnorm35_1_17_")
        conv33_1_18_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_18_")
        # conv33_1_18_, output shape: {[8,28,28]}

        batchnorm33_1_18_ = mx.symbol.BatchNorm(data=conv33_1_18_,
            fix_gamma=True,
            name="batchnorm33_1_18_")
        relu33_1_18_ = mx.symbol.Activation(data=batchnorm33_1_18_,
            act_type='relu',
            name="relu33_1_18_")

        conv34_1_18_ = mx.symbol.pad(data=relu33_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_18_ = mx.symbol.Convolution(data=conv34_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_18_")
        # conv34_1_18_, output shape: {[8,28,28]}

        batchnorm34_1_18_ = mx.symbol.BatchNorm(data=conv34_1_18_,
            fix_gamma=True,
            name="batchnorm34_1_18_")
        relu34_1_18_ = mx.symbol.Activation(data=batchnorm34_1_18_,
            act_type='relu',
            name="relu34_1_18_")

        conv35_1_18_ = mx.symbol.Convolution(data=relu34_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_18_")
        # conv35_1_18_, output shape: {[512,28,28]}

        batchnorm35_1_18_ = mx.symbol.BatchNorm(data=conv35_1_18_,
            fix_gamma=True,
            name="batchnorm35_1_18_")
        conv33_1_19_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_19_")
        # conv33_1_19_, output shape: {[8,28,28]}

        batchnorm33_1_19_ = mx.symbol.BatchNorm(data=conv33_1_19_,
            fix_gamma=True,
            name="batchnorm33_1_19_")
        relu33_1_19_ = mx.symbol.Activation(data=batchnorm33_1_19_,
            act_type='relu',
            name="relu33_1_19_")

        conv34_1_19_ = mx.symbol.pad(data=relu33_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_19_ = mx.symbol.Convolution(data=conv34_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_19_")
        # conv34_1_19_, output shape: {[8,28,28]}

        batchnorm34_1_19_ = mx.symbol.BatchNorm(data=conv34_1_19_,
            fix_gamma=True,
            name="batchnorm34_1_19_")
        relu34_1_19_ = mx.symbol.Activation(data=batchnorm34_1_19_,
            act_type='relu',
            name="relu34_1_19_")

        conv35_1_19_ = mx.symbol.Convolution(data=relu34_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_19_")
        # conv35_1_19_, output shape: {[512,28,28]}

        batchnorm35_1_19_ = mx.symbol.BatchNorm(data=conv35_1_19_,
            fix_gamma=True,
            name="batchnorm35_1_19_")
        conv33_1_20_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_20_")
        # conv33_1_20_, output shape: {[8,28,28]}

        batchnorm33_1_20_ = mx.symbol.BatchNorm(data=conv33_1_20_,
            fix_gamma=True,
            name="batchnorm33_1_20_")
        relu33_1_20_ = mx.symbol.Activation(data=batchnorm33_1_20_,
            act_type='relu',
            name="relu33_1_20_")

        conv34_1_20_ = mx.symbol.pad(data=relu33_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_20_ = mx.symbol.Convolution(data=conv34_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_20_")
        # conv34_1_20_, output shape: {[8,28,28]}

        batchnorm34_1_20_ = mx.symbol.BatchNorm(data=conv34_1_20_,
            fix_gamma=True,
            name="batchnorm34_1_20_")
        relu34_1_20_ = mx.symbol.Activation(data=batchnorm34_1_20_,
            act_type='relu',
            name="relu34_1_20_")

        conv35_1_20_ = mx.symbol.Convolution(data=relu34_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_20_")
        # conv35_1_20_, output shape: {[512,28,28]}

        batchnorm35_1_20_ = mx.symbol.BatchNorm(data=conv35_1_20_,
            fix_gamma=True,
            name="batchnorm35_1_20_")
        conv33_1_21_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_21_")
        # conv33_1_21_, output shape: {[8,28,28]}

        batchnorm33_1_21_ = mx.symbol.BatchNorm(data=conv33_1_21_,
            fix_gamma=True,
            name="batchnorm33_1_21_")
        relu33_1_21_ = mx.symbol.Activation(data=batchnorm33_1_21_,
            act_type='relu',
            name="relu33_1_21_")

        conv34_1_21_ = mx.symbol.pad(data=relu33_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_21_ = mx.symbol.Convolution(data=conv34_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_21_")
        # conv34_1_21_, output shape: {[8,28,28]}

        batchnorm34_1_21_ = mx.symbol.BatchNorm(data=conv34_1_21_,
            fix_gamma=True,
            name="batchnorm34_1_21_")
        relu34_1_21_ = mx.symbol.Activation(data=batchnorm34_1_21_,
            act_type='relu',
            name="relu34_1_21_")

        conv35_1_21_ = mx.symbol.Convolution(data=relu34_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_21_")
        # conv35_1_21_, output shape: {[512,28,28]}

        batchnorm35_1_21_ = mx.symbol.BatchNorm(data=conv35_1_21_,
            fix_gamma=True,
            name="batchnorm35_1_21_")
        conv33_1_22_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_22_")
        # conv33_1_22_, output shape: {[8,28,28]}

        batchnorm33_1_22_ = mx.symbol.BatchNorm(data=conv33_1_22_,
            fix_gamma=True,
            name="batchnorm33_1_22_")
        relu33_1_22_ = mx.symbol.Activation(data=batchnorm33_1_22_,
            act_type='relu',
            name="relu33_1_22_")

        conv34_1_22_ = mx.symbol.pad(data=relu33_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_22_ = mx.symbol.Convolution(data=conv34_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_22_")
        # conv34_1_22_, output shape: {[8,28,28]}

        batchnorm34_1_22_ = mx.symbol.BatchNorm(data=conv34_1_22_,
            fix_gamma=True,
            name="batchnorm34_1_22_")
        relu34_1_22_ = mx.symbol.Activation(data=batchnorm34_1_22_,
            act_type='relu',
            name="relu34_1_22_")

        conv35_1_22_ = mx.symbol.Convolution(data=relu34_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_22_")
        # conv35_1_22_, output shape: {[512,28,28]}

        batchnorm35_1_22_ = mx.symbol.BatchNorm(data=conv35_1_22_,
            fix_gamma=True,
            name="batchnorm35_1_22_")
        conv33_1_23_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_23_")
        # conv33_1_23_, output shape: {[8,28,28]}

        batchnorm33_1_23_ = mx.symbol.BatchNorm(data=conv33_1_23_,
            fix_gamma=True,
            name="batchnorm33_1_23_")
        relu33_1_23_ = mx.symbol.Activation(data=batchnorm33_1_23_,
            act_type='relu',
            name="relu33_1_23_")

        conv34_1_23_ = mx.symbol.pad(data=relu33_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_23_ = mx.symbol.Convolution(data=conv34_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_23_")
        # conv34_1_23_, output shape: {[8,28,28]}

        batchnorm34_1_23_ = mx.symbol.BatchNorm(data=conv34_1_23_,
            fix_gamma=True,
            name="batchnorm34_1_23_")
        relu34_1_23_ = mx.symbol.Activation(data=batchnorm34_1_23_,
            act_type='relu',
            name="relu34_1_23_")

        conv35_1_23_ = mx.symbol.Convolution(data=relu34_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_23_")
        # conv35_1_23_, output shape: {[512,28,28]}

        batchnorm35_1_23_ = mx.symbol.BatchNorm(data=conv35_1_23_,
            fix_gamma=True,
            name="batchnorm35_1_23_")
        conv33_1_24_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_24_")
        # conv33_1_24_, output shape: {[8,28,28]}

        batchnorm33_1_24_ = mx.symbol.BatchNorm(data=conv33_1_24_,
            fix_gamma=True,
            name="batchnorm33_1_24_")
        relu33_1_24_ = mx.symbol.Activation(data=batchnorm33_1_24_,
            act_type='relu',
            name="relu33_1_24_")

        conv34_1_24_ = mx.symbol.pad(data=relu33_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_24_ = mx.symbol.Convolution(data=conv34_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_24_")
        # conv34_1_24_, output shape: {[8,28,28]}

        batchnorm34_1_24_ = mx.symbol.BatchNorm(data=conv34_1_24_,
            fix_gamma=True,
            name="batchnorm34_1_24_")
        relu34_1_24_ = mx.symbol.Activation(data=batchnorm34_1_24_,
            act_type='relu',
            name="relu34_1_24_")

        conv35_1_24_ = mx.symbol.Convolution(data=relu34_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_24_")
        # conv35_1_24_, output shape: {[512,28,28]}

        batchnorm35_1_24_ = mx.symbol.BatchNorm(data=conv35_1_24_,
            fix_gamma=True,
            name="batchnorm35_1_24_")
        conv33_1_25_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_25_")
        # conv33_1_25_, output shape: {[8,28,28]}

        batchnorm33_1_25_ = mx.symbol.BatchNorm(data=conv33_1_25_,
            fix_gamma=True,
            name="batchnorm33_1_25_")
        relu33_1_25_ = mx.symbol.Activation(data=batchnorm33_1_25_,
            act_type='relu',
            name="relu33_1_25_")

        conv34_1_25_ = mx.symbol.pad(data=relu33_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_25_ = mx.symbol.Convolution(data=conv34_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_25_")
        # conv34_1_25_, output shape: {[8,28,28]}

        batchnorm34_1_25_ = mx.symbol.BatchNorm(data=conv34_1_25_,
            fix_gamma=True,
            name="batchnorm34_1_25_")
        relu34_1_25_ = mx.symbol.Activation(data=batchnorm34_1_25_,
            act_type='relu',
            name="relu34_1_25_")

        conv35_1_25_ = mx.symbol.Convolution(data=relu34_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_25_")
        # conv35_1_25_, output shape: {[512,28,28]}

        batchnorm35_1_25_ = mx.symbol.BatchNorm(data=conv35_1_25_,
            fix_gamma=True,
            name="batchnorm35_1_25_")
        conv33_1_26_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_26_")
        # conv33_1_26_, output shape: {[8,28,28]}

        batchnorm33_1_26_ = mx.symbol.BatchNorm(data=conv33_1_26_,
            fix_gamma=True,
            name="batchnorm33_1_26_")
        relu33_1_26_ = mx.symbol.Activation(data=batchnorm33_1_26_,
            act_type='relu',
            name="relu33_1_26_")

        conv34_1_26_ = mx.symbol.pad(data=relu33_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_26_ = mx.symbol.Convolution(data=conv34_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_26_")
        # conv34_1_26_, output shape: {[8,28,28]}

        batchnorm34_1_26_ = mx.symbol.BatchNorm(data=conv34_1_26_,
            fix_gamma=True,
            name="batchnorm34_1_26_")
        relu34_1_26_ = mx.symbol.Activation(data=batchnorm34_1_26_,
            act_type='relu',
            name="relu34_1_26_")

        conv35_1_26_ = mx.symbol.Convolution(data=relu34_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_26_")
        # conv35_1_26_, output shape: {[512,28,28]}

        batchnorm35_1_26_ = mx.symbol.BatchNorm(data=conv35_1_26_,
            fix_gamma=True,
            name="batchnorm35_1_26_")
        conv33_1_27_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_27_")
        # conv33_1_27_, output shape: {[8,28,28]}

        batchnorm33_1_27_ = mx.symbol.BatchNorm(data=conv33_1_27_,
            fix_gamma=True,
            name="batchnorm33_1_27_")
        relu33_1_27_ = mx.symbol.Activation(data=batchnorm33_1_27_,
            act_type='relu',
            name="relu33_1_27_")

        conv34_1_27_ = mx.symbol.pad(data=relu33_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_27_ = mx.symbol.Convolution(data=conv34_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_27_")
        # conv34_1_27_, output shape: {[8,28,28]}

        batchnorm34_1_27_ = mx.symbol.BatchNorm(data=conv34_1_27_,
            fix_gamma=True,
            name="batchnorm34_1_27_")
        relu34_1_27_ = mx.symbol.Activation(data=batchnorm34_1_27_,
            act_type='relu',
            name="relu34_1_27_")

        conv35_1_27_ = mx.symbol.Convolution(data=relu34_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_27_")
        # conv35_1_27_, output shape: {[512,28,28]}

        batchnorm35_1_27_ = mx.symbol.BatchNorm(data=conv35_1_27_,
            fix_gamma=True,
            name="batchnorm35_1_27_")
        conv33_1_28_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_28_")
        # conv33_1_28_, output shape: {[8,28,28]}

        batchnorm33_1_28_ = mx.symbol.BatchNorm(data=conv33_1_28_,
            fix_gamma=True,
            name="batchnorm33_1_28_")
        relu33_1_28_ = mx.symbol.Activation(data=batchnorm33_1_28_,
            act_type='relu',
            name="relu33_1_28_")

        conv34_1_28_ = mx.symbol.pad(data=relu33_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_28_ = mx.symbol.Convolution(data=conv34_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_28_")
        # conv34_1_28_, output shape: {[8,28,28]}

        batchnorm34_1_28_ = mx.symbol.BatchNorm(data=conv34_1_28_,
            fix_gamma=True,
            name="batchnorm34_1_28_")
        relu34_1_28_ = mx.symbol.Activation(data=batchnorm34_1_28_,
            act_type='relu',
            name="relu34_1_28_")

        conv35_1_28_ = mx.symbol.Convolution(data=relu34_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_28_")
        # conv35_1_28_, output shape: {[512,28,28]}

        batchnorm35_1_28_ = mx.symbol.BatchNorm(data=conv35_1_28_,
            fix_gamma=True,
            name="batchnorm35_1_28_")
        conv33_1_29_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_29_")
        # conv33_1_29_, output shape: {[8,28,28]}

        batchnorm33_1_29_ = mx.symbol.BatchNorm(data=conv33_1_29_,
            fix_gamma=True,
            name="batchnorm33_1_29_")
        relu33_1_29_ = mx.symbol.Activation(data=batchnorm33_1_29_,
            act_type='relu',
            name="relu33_1_29_")

        conv34_1_29_ = mx.symbol.pad(data=relu33_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_29_ = mx.symbol.Convolution(data=conv34_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_29_")
        # conv34_1_29_, output shape: {[8,28,28]}

        batchnorm34_1_29_ = mx.symbol.BatchNorm(data=conv34_1_29_,
            fix_gamma=True,
            name="batchnorm34_1_29_")
        relu34_1_29_ = mx.symbol.Activation(data=batchnorm34_1_29_,
            act_type='relu',
            name="relu34_1_29_")

        conv35_1_29_ = mx.symbol.Convolution(data=relu34_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_29_")
        # conv35_1_29_, output shape: {[512,28,28]}

        batchnorm35_1_29_ = mx.symbol.BatchNorm(data=conv35_1_29_,
            fix_gamma=True,
            name="batchnorm35_1_29_")
        conv33_1_30_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_30_")
        # conv33_1_30_, output shape: {[8,28,28]}

        batchnorm33_1_30_ = mx.symbol.BatchNorm(data=conv33_1_30_,
            fix_gamma=True,
            name="batchnorm33_1_30_")
        relu33_1_30_ = mx.symbol.Activation(data=batchnorm33_1_30_,
            act_type='relu',
            name="relu33_1_30_")

        conv34_1_30_ = mx.symbol.pad(data=relu33_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_30_ = mx.symbol.Convolution(data=conv34_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_30_")
        # conv34_1_30_, output shape: {[8,28,28]}

        batchnorm34_1_30_ = mx.symbol.BatchNorm(data=conv34_1_30_,
            fix_gamma=True,
            name="batchnorm34_1_30_")
        relu34_1_30_ = mx.symbol.Activation(data=batchnorm34_1_30_,
            act_type='relu',
            name="relu34_1_30_")

        conv35_1_30_ = mx.symbol.Convolution(data=relu34_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_30_")
        # conv35_1_30_, output shape: {[512,28,28]}

        batchnorm35_1_30_ = mx.symbol.BatchNorm(data=conv35_1_30_,
            fix_gamma=True,
            name="batchnorm35_1_30_")
        conv33_1_31_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_31_")
        # conv33_1_31_, output shape: {[8,28,28]}

        batchnorm33_1_31_ = mx.symbol.BatchNorm(data=conv33_1_31_,
            fix_gamma=True,
            name="batchnorm33_1_31_")
        relu33_1_31_ = mx.symbol.Activation(data=batchnorm33_1_31_,
            act_type='relu',
            name="relu33_1_31_")

        conv34_1_31_ = mx.symbol.pad(data=relu33_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_31_ = mx.symbol.Convolution(data=conv34_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_31_")
        # conv34_1_31_, output shape: {[8,28,28]}

        batchnorm34_1_31_ = mx.symbol.BatchNorm(data=conv34_1_31_,
            fix_gamma=True,
            name="batchnorm34_1_31_")
        relu34_1_31_ = mx.symbol.Activation(data=batchnorm34_1_31_,
            act_type='relu',
            name="relu34_1_31_")

        conv35_1_31_ = mx.symbol.Convolution(data=relu34_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_31_")
        # conv35_1_31_, output shape: {[512,28,28]}

        batchnorm35_1_31_ = mx.symbol.BatchNorm(data=conv35_1_31_,
            fix_gamma=True,
            name="batchnorm35_1_31_")
        conv33_1_32_ = mx.symbol.Convolution(data=relu31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv33_1_32_")
        # conv33_1_32_, output shape: {[8,28,28]}

        batchnorm33_1_32_ = mx.symbol.BatchNorm(data=conv33_1_32_,
            fix_gamma=True,
            name="batchnorm33_1_32_")
        relu33_1_32_ = mx.symbol.Activation(data=batchnorm33_1_32_,
            act_type='relu',
            name="relu33_1_32_")

        conv34_1_32_ = mx.symbol.pad(data=relu33_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv34_1_32_ = mx.symbol.Convolution(data=conv34_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv34_1_32_")
        # conv34_1_32_, output shape: {[8,28,28]}

        batchnorm34_1_32_ = mx.symbol.BatchNorm(data=conv34_1_32_,
            fix_gamma=True,
            name="batchnorm34_1_32_")
        relu34_1_32_ = mx.symbol.Activation(data=batchnorm34_1_32_,
            act_type='relu',
            name="relu34_1_32_")

        conv35_1_32_ = mx.symbol.Convolution(data=relu34_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv35_1_32_")
        # conv35_1_32_, output shape: {[512,28,28]}

        batchnorm35_1_32_ = mx.symbol.BatchNorm(data=conv35_1_32_,
            fix_gamma=True,
            name="batchnorm35_1_32_")
        add36_1_ = batchnorm35_1_1_ + batchnorm35_1_2_ + batchnorm35_1_3_ + batchnorm35_1_4_ + batchnorm35_1_5_ + batchnorm35_1_6_ + batchnorm35_1_7_ + batchnorm35_1_8_ + batchnorm35_1_9_ + batchnorm35_1_10_ + batchnorm35_1_11_ + batchnorm35_1_12_ + batchnorm35_1_13_ + batchnorm35_1_14_ + batchnorm35_1_15_ + batchnorm35_1_16_ + batchnorm35_1_17_ + batchnorm35_1_18_ + batchnorm35_1_19_ + batchnorm35_1_20_ + batchnorm35_1_21_ + batchnorm35_1_22_ + batchnorm35_1_23_ + batchnorm35_1_24_ + batchnorm35_1_25_ + batchnorm35_1_26_ + batchnorm35_1_27_ + batchnorm35_1_28_ + batchnorm35_1_29_ + batchnorm35_1_30_ + batchnorm35_1_31_ + batchnorm35_1_32_
        # add36_1_, output shape: {[512,28,28]}

        add37_ = add36_1_ + relu31_
        # add37_, output shape: {[512,28,28]}

        relu37_ = mx.symbol.Activation(data=add37_,
            act_type='relu',
            name="relu37_")

        conv39_1_1_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_1_")
        # conv39_1_1_, output shape: {[8,28,28]}

        batchnorm39_1_1_ = mx.symbol.BatchNorm(data=conv39_1_1_,
            fix_gamma=True,
            name="batchnorm39_1_1_")
        relu39_1_1_ = mx.symbol.Activation(data=batchnorm39_1_1_,
            act_type='relu',
            name="relu39_1_1_")

        conv40_1_1_ = mx.symbol.pad(data=relu39_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_1_ = mx.symbol.Convolution(data=conv40_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_1_")
        # conv40_1_1_, output shape: {[8,28,28]}

        batchnorm40_1_1_ = mx.symbol.BatchNorm(data=conv40_1_1_,
            fix_gamma=True,
            name="batchnorm40_1_1_")
        relu40_1_1_ = mx.symbol.Activation(data=batchnorm40_1_1_,
            act_type='relu',
            name="relu40_1_1_")

        conv41_1_1_ = mx.symbol.Convolution(data=relu40_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_1_")
        # conv41_1_1_, output shape: {[512,28,28]}

        batchnorm41_1_1_ = mx.symbol.BatchNorm(data=conv41_1_1_,
            fix_gamma=True,
            name="batchnorm41_1_1_")
        conv39_1_2_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_2_")
        # conv39_1_2_, output shape: {[8,28,28]}

        batchnorm39_1_2_ = mx.symbol.BatchNorm(data=conv39_1_2_,
            fix_gamma=True,
            name="batchnorm39_1_2_")
        relu39_1_2_ = mx.symbol.Activation(data=batchnorm39_1_2_,
            act_type='relu',
            name="relu39_1_2_")

        conv40_1_2_ = mx.symbol.pad(data=relu39_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_2_ = mx.symbol.Convolution(data=conv40_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_2_")
        # conv40_1_2_, output shape: {[8,28,28]}

        batchnorm40_1_2_ = mx.symbol.BatchNorm(data=conv40_1_2_,
            fix_gamma=True,
            name="batchnorm40_1_2_")
        relu40_1_2_ = mx.symbol.Activation(data=batchnorm40_1_2_,
            act_type='relu',
            name="relu40_1_2_")

        conv41_1_2_ = mx.symbol.Convolution(data=relu40_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_2_")
        # conv41_1_2_, output shape: {[512,28,28]}

        batchnorm41_1_2_ = mx.symbol.BatchNorm(data=conv41_1_2_,
            fix_gamma=True,
            name="batchnorm41_1_2_")
        conv39_1_3_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_3_")
        # conv39_1_3_, output shape: {[8,28,28]}

        batchnorm39_1_3_ = mx.symbol.BatchNorm(data=conv39_1_3_,
            fix_gamma=True,
            name="batchnorm39_1_3_")
        relu39_1_3_ = mx.symbol.Activation(data=batchnorm39_1_3_,
            act_type='relu',
            name="relu39_1_3_")

        conv40_1_3_ = mx.symbol.pad(data=relu39_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_3_ = mx.symbol.Convolution(data=conv40_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_3_")
        # conv40_1_3_, output shape: {[8,28,28]}

        batchnorm40_1_3_ = mx.symbol.BatchNorm(data=conv40_1_3_,
            fix_gamma=True,
            name="batchnorm40_1_3_")
        relu40_1_3_ = mx.symbol.Activation(data=batchnorm40_1_3_,
            act_type='relu',
            name="relu40_1_3_")

        conv41_1_3_ = mx.symbol.Convolution(data=relu40_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_3_")
        # conv41_1_3_, output shape: {[512,28,28]}

        batchnorm41_1_3_ = mx.symbol.BatchNorm(data=conv41_1_3_,
            fix_gamma=True,
            name="batchnorm41_1_3_")
        conv39_1_4_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_4_")
        # conv39_1_4_, output shape: {[8,28,28]}

        batchnorm39_1_4_ = mx.symbol.BatchNorm(data=conv39_1_4_,
            fix_gamma=True,
            name="batchnorm39_1_4_")
        relu39_1_4_ = mx.symbol.Activation(data=batchnorm39_1_4_,
            act_type='relu',
            name="relu39_1_4_")

        conv40_1_4_ = mx.symbol.pad(data=relu39_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_4_ = mx.symbol.Convolution(data=conv40_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_4_")
        # conv40_1_4_, output shape: {[8,28,28]}

        batchnorm40_1_4_ = mx.symbol.BatchNorm(data=conv40_1_4_,
            fix_gamma=True,
            name="batchnorm40_1_4_")
        relu40_1_4_ = mx.symbol.Activation(data=batchnorm40_1_4_,
            act_type='relu',
            name="relu40_1_4_")

        conv41_1_4_ = mx.symbol.Convolution(data=relu40_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_4_")
        # conv41_1_4_, output shape: {[512,28,28]}

        batchnorm41_1_4_ = mx.symbol.BatchNorm(data=conv41_1_4_,
            fix_gamma=True,
            name="batchnorm41_1_4_")
        conv39_1_5_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_5_")
        # conv39_1_5_, output shape: {[8,28,28]}

        batchnorm39_1_5_ = mx.symbol.BatchNorm(data=conv39_1_5_,
            fix_gamma=True,
            name="batchnorm39_1_5_")
        relu39_1_5_ = mx.symbol.Activation(data=batchnorm39_1_5_,
            act_type='relu',
            name="relu39_1_5_")

        conv40_1_5_ = mx.symbol.pad(data=relu39_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_5_ = mx.symbol.Convolution(data=conv40_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_5_")
        # conv40_1_5_, output shape: {[8,28,28]}

        batchnorm40_1_5_ = mx.symbol.BatchNorm(data=conv40_1_5_,
            fix_gamma=True,
            name="batchnorm40_1_5_")
        relu40_1_5_ = mx.symbol.Activation(data=batchnorm40_1_5_,
            act_type='relu',
            name="relu40_1_5_")

        conv41_1_5_ = mx.symbol.Convolution(data=relu40_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_5_")
        # conv41_1_5_, output shape: {[512,28,28]}

        batchnorm41_1_5_ = mx.symbol.BatchNorm(data=conv41_1_5_,
            fix_gamma=True,
            name="batchnorm41_1_5_")
        conv39_1_6_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_6_")
        # conv39_1_6_, output shape: {[8,28,28]}

        batchnorm39_1_6_ = mx.symbol.BatchNorm(data=conv39_1_6_,
            fix_gamma=True,
            name="batchnorm39_1_6_")
        relu39_1_6_ = mx.symbol.Activation(data=batchnorm39_1_6_,
            act_type='relu',
            name="relu39_1_6_")

        conv40_1_6_ = mx.symbol.pad(data=relu39_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_6_ = mx.symbol.Convolution(data=conv40_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_6_")
        # conv40_1_6_, output shape: {[8,28,28]}

        batchnorm40_1_6_ = mx.symbol.BatchNorm(data=conv40_1_6_,
            fix_gamma=True,
            name="batchnorm40_1_6_")
        relu40_1_6_ = mx.symbol.Activation(data=batchnorm40_1_6_,
            act_type='relu',
            name="relu40_1_6_")

        conv41_1_6_ = mx.symbol.Convolution(data=relu40_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_6_")
        # conv41_1_6_, output shape: {[512,28,28]}

        batchnorm41_1_6_ = mx.symbol.BatchNorm(data=conv41_1_6_,
            fix_gamma=True,
            name="batchnorm41_1_6_")
        conv39_1_7_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_7_")
        # conv39_1_7_, output shape: {[8,28,28]}

        batchnorm39_1_7_ = mx.symbol.BatchNorm(data=conv39_1_7_,
            fix_gamma=True,
            name="batchnorm39_1_7_")
        relu39_1_7_ = mx.symbol.Activation(data=batchnorm39_1_7_,
            act_type='relu',
            name="relu39_1_7_")

        conv40_1_7_ = mx.symbol.pad(data=relu39_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_7_ = mx.symbol.Convolution(data=conv40_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_7_")
        # conv40_1_7_, output shape: {[8,28,28]}

        batchnorm40_1_7_ = mx.symbol.BatchNorm(data=conv40_1_7_,
            fix_gamma=True,
            name="batchnorm40_1_7_")
        relu40_1_7_ = mx.symbol.Activation(data=batchnorm40_1_7_,
            act_type='relu',
            name="relu40_1_7_")

        conv41_1_7_ = mx.symbol.Convolution(data=relu40_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_7_")
        # conv41_1_7_, output shape: {[512,28,28]}

        batchnorm41_1_7_ = mx.symbol.BatchNorm(data=conv41_1_7_,
            fix_gamma=True,
            name="batchnorm41_1_7_")
        conv39_1_8_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_8_")
        # conv39_1_8_, output shape: {[8,28,28]}

        batchnorm39_1_8_ = mx.symbol.BatchNorm(data=conv39_1_8_,
            fix_gamma=True,
            name="batchnorm39_1_8_")
        relu39_1_8_ = mx.symbol.Activation(data=batchnorm39_1_8_,
            act_type='relu',
            name="relu39_1_8_")

        conv40_1_8_ = mx.symbol.pad(data=relu39_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_8_ = mx.symbol.Convolution(data=conv40_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_8_")
        # conv40_1_8_, output shape: {[8,28,28]}

        batchnorm40_1_8_ = mx.symbol.BatchNorm(data=conv40_1_8_,
            fix_gamma=True,
            name="batchnorm40_1_8_")
        relu40_1_8_ = mx.symbol.Activation(data=batchnorm40_1_8_,
            act_type='relu',
            name="relu40_1_8_")

        conv41_1_8_ = mx.symbol.Convolution(data=relu40_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_8_")
        # conv41_1_8_, output shape: {[512,28,28]}

        batchnorm41_1_8_ = mx.symbol.BatchNorm(data=conv41_1_8_,
            fix_gamma=True,
            name="batchnorm41_1_8_")
        conv39_1_9_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_9_")
        # conv39_1_9_, output shape: {[8,28,28]}

        batchnorm39_1_9_ = mx.symbol.BatchNorm(data=conv39_1_9_,
            fix_gamma=True,
            name="batchnorm39_1_9_")
        relu39_1_9_ = mx.symbol.Activation(data=batchnorm39_1_9_,
            act_type='relu',
            name="relu39_1_9_")

        conv40_1_9_ = mx.symbol.pad(data=relu39_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_9_ = mx.symbol.Convolution(data=conv40_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_9_")
        # conv40_1_9_, output shape: {[8,28,28]}

        batchnorm40_1_9_ = mx.symbol.BatchNorm(data=conv40_1_9_,
            fix_gamma=True,
            name="batchnorm40_1_9_")
        relu40_1_9_ = mx.symbol.Activation(data=batchnorm40_1_9_,
            act_type='relu',
            name="relu40_1_9_")

        conv41_1_9_ = mx.symbol.Convolution(data=relu40_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_9_")
        # conv41_1_9_, output shape: {[512,28,28]}

        batchnorm41_1_9_ = mx.symbol.BatchNorm(data=conv41_1_9_,
            fix_gamma=True,
            name="batchnorm41_1_9_")
        conv39_1_10_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_10_")
        # conv39_1_10_, output shape: {[8,28,28]}

        batchnorm39_1_10_ = mx.symbol.BatchNorm(data=conv39_1_10_,
            fix_gamma=True,
            name="batchnorm39_1_10_")
        relu39_1_10_ = mx.symbol.Activation(data=batchnorm39_1_10_,
            act_type='relu',
            name="relu39_1_10_")

        conv40_1_10_ = mx.symbol.pad(data=relu39_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_10_ = mx.symbol.Convolution(data=conv40_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_10_")
        # conv40_1_10_, output shape: {[8,28,28]}

        batchnorm40_1_10_ = mx.symbol.BatchNorm(data=conv40_1_10_,
            fix_gamma=True,
            name="batchnorm40_1_10_")
        relu40_1_10_ = mx.symbol.Activation(data=batchnorm40_1_10_,
            act_type='relu',
            name="relu40_1_10_")

        conv41_1_10_ = mx.symbol.Convolution(data=relu40_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_10_")
        # conv41_1_10_, output shape: {[512,28,28]}

        batchnorm41_1_10_ = mx.symbol.BatchNorm(data=conv41_1_10_,
            fix_gamma=True,
            name="batchnorm41_1_10_")
        conv39_1_11_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_11_")
        # conv39_1_11_, output shape: {[8,28,28]}

        batchnorm39_1_11_ = mx.symbol.BatchNorm(data=conv39_1_11_,
            fix_gamma=True,
            name="batchnorm39_1_11_")
        relu39_1_11_ = mx.symbol.Activation(data=batchnorm39_1_11_,
            act_type='relu',
            name="relu39_1_11_")

        conv40_1_11_ = mx.symbol.pad(data=relu39_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_11_ = mx.symbol.Convolution(data=conv40_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_11_")
        # conv40_1_11_, output shape: {[8,28,28]}

        batchnorm40_1_11_ = mx.symbol.BatchNorm(data=conv40_1_11_,
            fix_gamma=True,
            name="batchnorm40_1_11_")
        relu40_1_11_ = mx.symbol.Activation(data=batchnorm40_1_11_,
            act_type='relu',
            name="relu40_1_11_")

        conv41_1_11_ = mx.symbol.Convolution(data=relu40_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_11_")
        # conv41_1_11_, output shape: {[512,28,28]}

        batchnorm41_1_11_ = mx.symbol.BatchNorm(data=conv41_1_11_,
            fix_gamma=True,
            name="batchnorm41_1_11_")
        conv39_1_12_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_12_")
        # conv39_1_12_, output shape: {[8,28,28]}

        batchnorm39_1_12_ = mx.symbol.BatchNorm(data=conv39_1_12_,
            fix_gamma=True,
            name="batchnorm39_1_12_")
        relu39_1_12_ = mx.symbol.Activation(data=batchnorm39_1_12_,
            act_type='relu',
            name="relu39_1_12_")

        conv40_1_12_ = mx.symbol.pad(data=relu39_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_12_ = mx.symbol.Convolution(data=conv40_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_12_")
        # conv40_1_12_, output shape: {[8,28,28]}

        batchnorm40_1_12_ = mx.symbol.BatchNorm(data=conv40_1_12_,
            fix_gamma=True,
            name="batchnorm40_1_12_")
        relu40_1_12_ = mx.symbol.Activation(data=batchnorm40_1_12_,
            act_type='relu',
            name="relu40_1_12_")

        conv41_1_12_ = mx.symbol.Convolution(data=relu40_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_12_")
        # conv41_1_12_, output shape: {[512,28,28]}

        batchnorm41_1_12_ = mx.symbol.BatchNorm(data=conv41_1_12_,
            fix_gamma=True,
            name="batchnorm41_1_12_")
        conv39_1_13_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_13_")
        # conv39_1_13_, output shape: {[8,28,28]}

        batchnorm39_1_13_ = mx.symbol.BatchNorm(data=conv39_1_13_,
            fix_gamma=True,
            name="batchnorm39_1_13_")
        relu39_1_13_ = mx.symbol.Activation(data=batchnorm39_1_13_,
            act_type='relu',
            name="relu39_1_13_")

        conv40_1_13_ = mx.symbol.pad(data=relu39_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_13_ = mx.symbol.Convolution(data=conv40_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_13_")
        # conv40_1_13_, output shape: {[8,28,28]}

        batchnorm40_1_13_ = mx.symbol.BatchNorm(data=conv40_1_13_,
            fix_gamma=True,
            name="batchnorm40_1_13_")
        relu40_1_13_ = mx.symbol.Activation(data=batchnorm40_1_13_,
            act_type='relu',
            name="relu40_1_13_")

        conv41_1_13_ = mx.symbol.Convolution(data=relu40_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_13_")
        # conv41_1_13_, output shape: {[512,28,28]}

        batchnorm41_1_13_ = mx.symbol.BatchNorm(data=conv41_1_13_,
            fix_gamma=True,
            name="batchnorm41_1_13_")
        conv39_1_14_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_14_")
        # conv39_1_14_, output shape: {[8,28,28]}

        batchnorm39_1_14_ = mx.symbol.BatchNorm(data=conv39_1_14_,
            fix_gamma=True,
            name="batchnorm39_1_14_")
        relu39_1_14_ = mx.symbol.Activation(data=batchnorm39_1_14_,
            act_type='relu',
            name="relu39_1_14_")

        conv40_1_14_ = mx.symbol.pad(data=relu39_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_14_ = mx.symbol.Convolution(data=conv40_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_14_")
        # conv40_1_14_, output shape: {[8,28,28]}

        batchnorm40_1_14_ = mx.symbol.BatchNorm(data=conv40_1_14_,
            fix_gamma=True,
            name="batchnorm40_1_14_")
        relu40_1_14_ = mx.symbol.Activation(data=batchnorm40_1_14_,
            act_type='relu',
            name="relu40_1_14_")

        conv41_1_14_ = mx.symbol.Convolution(data=relu40_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_14_")
        # conv41_1_14_, output shape: {[512,28,28]}

        batchnorm41_1_14_ = mx.symbol.BatchNorm(data=conv41_1_14_,
            fix_gamma=True,
            name="batchnorm41_1_14_")
        conv39_1_15_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_15_")
        # conv39_1_15_, output shape: {[8,28,28]}

        batchnorm39_1_15_ = mx.symbol.BatchNorm(data=conv39_1_15_,
            fix_gamma=True,
            name="batchnorm39_1_15_")
        relu39_1_15_ = mx.symbol.Activation(data=batchnorm39_1_15_,
            act_type='relu',
            name="relu39_1_15_")

        conv40_1_15_ = mx.symbol.pad(data=relu39_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_15_ = mx.symbol.Convolution(data=conv40_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_15_")
        # conv40_1_15_, output shape: {[8,28,28]}

        batchnorm40_1_15_ = mx.symbol.BatchNorm(data=conv40_1_15_,
            fix_gamma=True,
            name="batchnorm40_1_15_")
        relu40_1_15_ = mx.symbol.Activation(data=batchnorm40_1_15_,
            act_type='relu',
            name="relu40_1_15_")

        conv41_1_15_ = mx.symbol.Convolution(data=relu40_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_15_")
        # conv41_1_15_, output shape: {[512,28,28]}

        batchnorm41_1_15_ = mx.symbol.BatchNorm(data=conv41_1_15_,
            fix_gamma=True,
            name="batchnorm41_1_15_")
        conv39_1_16_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_16_")
        # conv39_1_16_, output shape: {[8,28,28]}

        batchnorm39_1_16_ = mx.symbol.BatchNorm(data=conv39_1_16_,
            fix_gamma=True,
            name="batchnorm39_1_16_")
        relu39_1_16_ = mx.symbol.Activation(data=batchnorm39_1_16_,
            act_type='relu',
            name="relu39_1_16_")

        conv40_1_16_ = mx.symbol.pad(data=relu39_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_16_ = mx.symbol.Convolution(data=conv40_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_16_")
        # conv40_1_16_, output shape: {[8,28,28]}

        batchnorm40_1_16_ = mx.symbol.BatchNorm(data=conv40_1_16_,
            fix_gamma=True,
            name="batchnorm40_1_16_")
        relu40_1_16_ = mx.symbol.Activation(data=batchnorm40_1_16_,
            act_type='relu',
            name="relu40_1_16_")

        conv41_1_16_ = mx.symbol.Convolution(data=relu40_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_16_")
        # conv41_1_16_, output shape: {[512,28,28]}

        batchnorm41_1_16_ = mx.symbol.BatchNorm(data=conv41_1_16_,
            fix_gamma=True,
            name="batchnorm41_1_16_")
        conv39_1_17_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_17_")
        # conv39_1_17_, output shape: {[8,28,28]}

        batchnorm39_1_17_ = mx.symbol.BatchNorm(data=conv39_1_17_,
            fix_gamma=True,
            name="batchnorm39_1_17_")
        relu39_1_17_ = mx.symbol.Activation(data=batchnorm39_1_17_,
            act_type='relu',
            name="relu39_1_17_")

        conv40_1_17_ = mx.symbol.pad(data=relu39_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_17_ = mx.symbol.Convolution(data=conv40_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_17_")
        # conv40_1_17_, output shape: {[8,28,28]}

        batchnorm40_1_17_ = mx.symbol.BatchNorm(data=conv40_1_17_,
            fix_gamma=True,
            name="batchnorm40_1_17_")
        relu40_1_17_ = mx.symbol.Activation(data=batchnorm40_1_17_,
            act_type='relu',
            name="relu40_1_17_")

        conv41_1_17_ = mx.symbol.Convolution(data=relu40_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_17_")
        # conv41_1_17_, output shape: {[512,28,28]}

        batchnorm41_1_17_ = mx.symbol.BatchNorm(data=conv41_1_17_,
            fix_gamma=True,
            name="batchnorm41_1_17_")
        conv39_1_18_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_18_")
        # conv39_1_18_, output shape: {[8,28,28]}

        batchnorm39_1_18_ = mx.symbol.BatchNorm(data=conv39_1_18_,
            fix_gamma=True,
            name="batchnorm39_1_18_")
        relu39_1_18_ = mx.symbol.Activation(data=batchnorm39_1_18_,
            act_type='relu',
            name="relu39_1_18_")

        conv40_1_18_ = mx.symbol.pad(data=relu39_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_18_ = mx.symbol.Convolution(data=conv40_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_18_")
        # conv40_1_18_, output shape: {[8,28,28]}

        batchnorm40_1_18_ = mx.symbol.BatchNorm(data=conv40_1_18_,
            fix_gamma=True,
            name="batchnorm40_1_18_")
        relu40_1_18_ = mx.symbol.Activation(data=batchnorm40_1_18_,
            act_type='relu',
            name="relu40_1_18_")

        conv41_1_18_ = mx.symbol.Convolution(data=relu40_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_18_")
        # conv41_1_18_, output shape: {[512,28,28]}

        batchnorm41_1_18_ = mx.symbol.BatchNorm(data=conv41_1_18_,
            fix_gamma=True,
            name="batchnorm41_1_18_")
        conv39_1_19_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_19_")
        # conv39_1_19_, output shape: {[8,28,28]}

        batchnorm39_1_19_ = mx.symbol.BatchNorm(data=conv39_1_19_,
            fix_gamma=True,
            name="batchnorm39_1_19_")
        relu39_1_19_ = mx.symbol.Activation(data=batchnorm39_1_19_,
            act_type='relu',
            name="relu39_1_19_")

        conv40_1_19_ = mx.symbol.pad(data=relu39_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_19_ = mx.symbol.Convolution(data=conv40_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_19_")
        # conv40_1_19_, output shape: {[8,28,28]}

        batchnorm40_1_19_ = mx.symbol.BatchNorm(data=conv40_1_19_,
            fix_gamma=True,
            name="batchnorm40_1_19_")
        relu40_1_19_ = mx.symbol.Activation(data=batchnorm40_1_19_,
            act_type='relu',
            name="relu40_1_19_")

        conv41_1_19_ = mx.symbol.Convolution(data=relu40_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_19_")
        # conv41_1_19_, output shape: {[512,28,28]}

        batchnorm41_1_19_ = mx.symbol.BatchNorm(data=conv41_1_19_,
            fix_gamma=True,
            name="batchnorm41_1_19_")
        conv39_1_20_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_20_")
        # conv39_1_20_, output shape: {[8,28,28]}

        batchnorm39_1_20_ = mx.symbol.BatchNorm(data=conv39_1_20_,
            fix_gamma=True,
            name="batchnorm39_1_20_")
        relu39_1_20_ = mx.symbol.Activation(data=batchnorm39_1_20_,
            act_type='relu',
            name="relu39_1_20_")

        conv40_1_20_ = mx.symbol.pad(data=relu39_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_20_ = mx.symbol.Convolution(data=conv40_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_20_")
        # conv40_1_20_, output shape: {[8,28,28]}

        batchnorm40_1_20_ = mx.symbol.BatchNorm(data=conv40_1_20_,
            fix_gamma=True,
            name="batchnorm40_1_20_")
        relu40_1_20_ = mx.symbol.Activation(data=batchnorm40_1_20_,
            act_type='relu',
            name="relu40_1_20_")

        conv41_1_20_ = mx.symbol.Convolution(data=relu40_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_20_")
        # conv41_1_20_, output shape: {[512,28,28]}

        batchnorm41_1_20_ = mx.symbol.BatchNorm(data=conv41_1_20_,
            fix_gamma=True,
            name="batchnorm41_1_20_")
        conv39_1_21_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_21_")
        # conv39_1_21_, output shape: {[8,28,28]}

        batchnorm39_1_21_ = mx.symbol.BatchNorm(data=conv39_1_21_,
            fix_gamma=True,
            name="batchnorm39_1_21_")
        relu39_1_21_ = mx.symbol.Activation(data=batchnorm39_1_21_,
            act_type='relu',
            name="relu39_1_21_")

        conv40_1_21_ = mx.symbol.pad(data=relu39_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_21_ = mx.symbol.Convolution(data=conv40_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_21_")
        # conv40_1_21_, output shape: {[8,28,28]}

        batchnorm40_1_21_ = mx.symbol.BatchNorm(data=conv40_1_21_,
            fix_gamma=True,
            name="batchnorm40_1_21_")
        relu40_1_21_ = mx.symbol.Activation(data=batchnorm40_1_21_,
            act_type='relu',
            name="relu40_1_21_")

        conv41_1_21_ = mx.symbol.Convolution(data=relu40_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_21_")
        # conv41_1_21_, output shape: {[512,28,28]}

        batchnorm41_1_21_ = mx.symbol.BatchNorm(data=conv41_1_21_,
            fix_gamma=True,
            name="batchnorm41_1_21_")
        conv39_1_22_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_22_")
        # conv39_1_22_, output shape: {[8,28,28]}

        batchnorm39_1_22_ = mx.symbol.BatchNorm(data=conv39_1_22_,
            fix_gamma=True,
            name="batchnorm39_1_22_")
        relu39_1_22_ = mx.symbol.Activation(data=batchnorm39_1_22_,
            act_type='relu',
            name="relu39_1_22_")

        conv40_1_22_ = mx.symbol.pad(data=relu39_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_22_ = mx.symbol.Convolution(data=conv40_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_22_")
        # conv40_1_22_, output shape: {[8,28,28]}

        batchnorm40_1_22_ = mx.symbol.BatchNorm(data=conv40_1_22_,
            fix_gamma=True,
            name="batchnorm40_1_22_")
        relu40_1_22_ = mx.symbol.Activation(data=batchnorm40_1_22_,
            act_type='relu',
            name="relu40_1_22_")

        conv41_1_22_ = mx.symbol.Convolution(data=relu40_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_22_")
        # conv41_1_22_, output shape: {[512,28,28]}

        batchnorm41_1_22_ = mx.symbol.BatchNorm(data=conv41_1_22_,
            fix_gamma=True,
            name="batchnorm41_1_22_")
        conv39_1_23_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_23_")
        # conv39_1_23_, output shape: {[8,28,28]}

        batchnorm39_1_23_ = mx.symbol.BatchNorm(data=conv39_1_23_,
            fix_gamma=True,
            name="batchnorm39_1_23_")
        relu39_1_23_ = mx.symbol.Activation(data=batchnorm39_1_23_,
            act_type='relu',
            name="relu39_1_23_")

        conv40_1_23_ = mx.symbol.pad(data=relu39_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_23_ = mx.symbol.Convolution(data=conv40_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_23_")
        # conv40_1_23_, output shape: {[8,28,28]}

        batchnorm40_1_23_ = mx.symbol.BatchNorm(data=conv40_1_23_,
            fix_gamma=True,
            name="batchnorm40_1_23_")
        relu40_1_23_ = mx.symbol.Activation(data=batchnorm40_1_23_,
            act_type='relu',
            name="relu40_1_23_")

        conv41_1_23_ = mx.symbol.Convolution(data=relu40_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_23_")
        # conv41_1_23_, output shape: {[512,28,28]}

        batchnorm41_1_23_ = mx.symbol.BatchNorm(data=conv41_1_23_,
            fix_gamma=True,
            name="batchnorm41_1_23_")
        conv39_1_24_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_24_")
        # conv39_1_24_, output shape: {[8,28,28]}

        batchnorm39_1_24_ = mx.symbol.BatchNorm(data=conv39_1_24_,
            fix_gamma=True,
            name="batchnorm39_1_24_")
        relu39_1_24_ = mx.symbol.Activation(data=batchnorm39_1_24_,
            act_type='relu',
            name="relu39_1_24_")

        conv40_1_24_ = mx.symbol.pad(data=relu39_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_24_ = mx.symbol.Convolution(data=conv40_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_24_")
        # conv40_1_24_, output shape: {[8,28,28]}

        batchnorm40_1_24_ = mx.symbol.BatchNorm(data=conv40_1_24_,
            fix_gamma=True,
            name="batchnorm40_1_24_")
        relu40_1_24_ = mx.symbol.Activation(data=batchnorm40_1_24_,
            act_type='relu',
            name="relu40_1_24_")

        conv41_1_24_ = mx.symbol.Convolution(data=relu40_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_24_")
        # conv41_1_24_, output shape: {[512,28,28]}

        batchnorm41_1_24_ = mx.symbol.BatchNorm(data=conv41_1_24_,
            fix_gamma=True,
            name="batchnorm41_1_24_")
        conv39_1_25_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_25_")
        # conv39_1_25_, output shape: {[8,28,28]}

        batchnorm39_1_25_ = mx.symbol.BatchNorm(data=conv39_1_25_,
            fix_gamma=True,
            name="batchnorm39_1_25_")
        relu39_1_25_ = mx.symbol.Activation(data=batchnorm39_1_25_,
            act_type='relu',
            name="relu39_1_25_")

        conv40_1_25_ = mx.symbol.pad(data=relu39_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_25_ = mx.symbol.Convolution(data=conv40_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_25_")
        # conv40_1_25_, output shape: {[8,28,28]}

        batchnorm40_1_25_ = mx.symbol.BatchNorm(data=conv40_1_25_,
            fix_gamma=True,
            name="batchnorm40_1_25_")
        relu40_1_25_ = mx.symbol.Activation(data=batchnorm40_1_25_,
            act_type='relu',
            name="relu40_1_25_")

        conv41_1_25_ = mx.symbol.Convolution(data=relu40_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_25_")
        # conv41_1_25_, output shape: {[512,28,28]}

        batchnorm41_1_25_ = mx.symbol.BatchNorm(data=conv41_1_25_,
            fix_gamma=True,
            name="batchnorm41_1_25_")
        conv39_1_26_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_26_")
        # conv39_1_26_, output shape: {[8,28,28]}

        batchnorm39_1_26_ = mx.symbol.BatchNorm(data=conv39_1_26_,
            fix_gamma=True,
            name="batchnorm39_1_26_")
        relu39_1_26_ = mx.symbol.Activation(data=batchnorm39_1_26_,
            act_type='relu',
            name="relu39_1_26_")

        conv40_1_26_ = mx.symbol.pad(data=relu39_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_26_ = mx.symbol.Convolution(data=conv40_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_26_")
        # conv40_1_26_, output shape: {[8,28,28]}

        batchnorm40_1_26_ = mx.symbol.BatchNorm(data=conv40_1_26_,
            fix_gamma=True,
            name="batchnorm40_1_26_")
        relu40_1_26_ = mx.symbol.Activation(data=batchnorm40_1_26_,
            act_type='relu',
            name="relu40_1_26_")

        conv41_1_26_ = mx.symbol.Convolution(data=relu40_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_26_")
        # conv41_1_26_, output shape: {[512,28,28]}

        batchnorm41_1_26_ = mx.symbol.BatchNorm(data=conv41_1_26_,
            fix_gamma=True,
            name="batchnorm41_1_26_")
        conv39_1_27_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_27_")
        # conv39_1_27_, output shape: {[8,28,28]}

        batchnorm39_1_27_ = mx.symbol.BatchNorm(data=conv39_1_27_,
            fix_gamma=True,
            name="batchnorm39_1_27_")
        relu39_1_27_ = mx.symbol.Activation(data=batchnorm39_1_27_,
            act_type='relu',
            name="relu39_1_27_")

        conv40_1_27_ = mx.symbol.pad(data=relu39_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_27_ = mx.symbol.Convolution(data=conv40_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_27_")
        # conv40_1_27_, output shape: {[8,28,28]}

        batchnorm40_1_27_ = mx.symbol.BatchNorm(data=conv40_1_27_,
            fix_gamma=True,
            name="batchnorm40_1_27_")
        relu40_1_27_ = mx.symbol.Activation(data=batchnorm40_1_27_,
            act_type='relu',
            name="relu40_1_27_")

        conv41_1_27_ = mx.symbol.Convolution(data=relu40_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_27_")
        # conv41_1_27_, output shape: {[512,28,28]}

        batchnorm41_1_27_ = mx.symbol.BatchNorm(data=conv41_1_27_,
            fix_gamma=True,
            name="batchnorm41_1_27_")
        conv39_1_28_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_28_")
        # conv39_1_28_, output shape: {[8,28,28]}

        batchnorm39_1_28_ = mx.symbol.BatchNorm(data=conv39_1_28_,
            fix_gamma=True,
            name="batchnorm39_1_28_")
        relu39_1_28_ = mx.symbol.Activation(data=batchnorm39_1_28_,
            act_type='relu',
            name="relu39_1_28_")

        conv40_1_28_ = mx.symbol.pad(data=relu39_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_28_ = mx.symbol.Convolution(data=conv40_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_28_")
        # conv40_1_28_, output shape: {[8,28,28]}

        batchnorm40_1_28_ = mx.symbol.BatchNorm(data=conv40_1_28_,
            fix_gamma=True,
            name="batchnorm40_1_28_")
        relu40_1_28_ = mx.symbol.Activation(data=batchnorm40_1_28_,
            act_type='relu',
            name="relu40_1_28_")

        conv41_1_28_ = mx.symbol.Convolution(data=relu40_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_28_")
        # conv41_1_28_, output shape: {[512,28,28]}

        batchnorm41_1_28_ = mx.symbol.BatchNorm(data=conv41_1_28_,
            fix_gamma=True,
            name="batchnorm41_1_28_")
        conv39_1_29_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_29_")
        # conv39_1_29_, output shape: {[8,28,28]}

        batchnorm39_1_29_ = mx.symbol.BatchNorm(data=conv39_1_29_,
            fix_gamma=True,
            name="batchnorm39_1_29_")
        relu39_1_29_ = mx.symbol.Activation(data=batchnorm39_1_29_,
            act_type='relu',
            name="relu39_1_29_")

        conv40_1_29_ = mx.symbol.pad(data=relu39_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_29_ = mx.symbol.Convolution(data=conv40_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_29_")
        # conv40_1_29_, output shape: {[8,28,28]}

        batchnorm40_1_29_ = mx.symbol.BatchNorm(data=conv40_1_29_,
            fix_gamma=True,
            name="batchnorm40_1_29_")
        relu40_1_29_ = mx.symbol.Activation(data=batchnorm40_1_29_,
            act_type='relu',
            name="relu40_1_29_")

        conv41_1_29_ = mx.symbol.Convolution(data=relu40_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_29_")
        # conv41_1_29_, output shape: {[512,28,28]}

        batchnorm41_1_29_ = mx.symbol.BatchNorm(data=conv41_1_29_,
            fix_gamma=True,
            name="batchnorm41_1_29_")
        conv39_1_30_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_30_")
        # conv39_1_30_, output shape: {[8,28,28]}

        batchnorm39_1_30_ = mx.symbol.BatchNorm(data=conv39_1_30_,
            fix_gamma=True,
            name="batchnorm39_1_30_")
        relu39_1_30_ = mx.symbol.Activation(data=batchnorm39_1_30_,
            act_type='relu',
            name="relu39_1_30_")

        conv40_1_30_ = mx.symbol.pad(data=relu39_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_30_ = mx.symbol.Convolution(data=conv40_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_30_")
        # conv40_1_30_, output shape: {[8,28,28]}

        batchnorm40_1_30_ = mx.symbol.BatchNorm(data=conv40_1_30_,
            fix_gamma=True,
            name="batchnorm40_1_30_")
        relu40_1_30_ = mx.symbol.Activation(data=batchnorm40_1_30_,
            act_type='relu',
            name="relu40_1_30_")

        conv41_1_30_ = mx.symbol.Convolution(data=relu40_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_30_")
        # conv41_1_30_, output shape: {[512,28,28]}

        batchnorm41_1_30_ = mx.symbol.BatchNorm(data=conv41_1_30_,
            fix_gamma=True,
            name="batchnorm41_1_30_")
        conv39_1_31_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_31_")
        # conv39_1_31_, output shape: {[8,28,28]}

        batchnorm39_1_31_ = mx.symbol.BatchNorm(data=conv39_1_31_,
            fix_gamma=True,
            name="batchnorm39_1_31_")
        relu39_1_31_ = mx.symbol.Activation(data=batchnorm39_1_31_,
            act_type='relu',
            name="relu39_1_31_")

        conv40_1_31_ = mx.symbol.pad(data=relu39_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_31_ = mx.symbol.Convolution(data=conv40_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_31_")
        # conv40_1_31_, output shape: {[8,28,28]}

        batchnorm40_1_31_ = mx.symbol.BatchNorm(data=conv40_1_31_,
            fix_gamma=True,
            name="batchnorm40_1_31_")
        relu40_1_31_ = mx.symbol.Activation(data=batchnorm40_1_31_,
            act_type='relu',
            name="relu40_1_31_")

        conv41_1_31_ = mx.symbol.Convolution(data=relu40_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_31_")
        # conv41_1_31_, output shape: {[512,28,28]}

        batchnorm41_1_31_ = mx.symbol.BatchNorm(data=conv41_1_31_,
            fix_gamma=True,
            name="batchnorm41_1_31_")
        conv39_1_32_ = mx.symbol.Convolution(data=relu37_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv39_1_32_")
        # conv39_1_32_, output shape: {[8,28,28]}

        batchnorm39_1_32_ = mx.symbol.BatchNorm(data=conv39_1_32_,
            fix_gamma=True,
            name="batchnorm39_1_32_")
        relu39_1_32_ = mx.symbol.Activation(data=batchnorm39_1_32_,
            act_type='relu',
            name="relu39_1_32_")

        conv40_1_32_ = mx.symbol.pad(data=relu39_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv40_1_32_ = mx.symbol.Convolution(data=conv40_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=8,
            no_bias=False,
            name="conv40_1_32_")
        # conv40_1_32_, output shape: {[8,28,28]}

        batchnorm40_1_32_ = mx.symbol.BatchNorm(data=conv40_1_32_,
            fix_gamma=True,
            name="batchnorm40_1_32_")
        relu40_1_32_ = mx.symbol.Activation(data=batchnorm40_1_32_,
            act_type='relu',
            name="relu40_1_32_")

        conv41_1_32_ = mx.symbol.Convolution(data=relu40_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv41_1_32_")
        # conv41_1_32_, output shape: {[512,28,28]}

        batchnorm41_1_32_ = mx.symbol.BatchNorm(data=conv41_1_32_,
            fix_gamma=True,
            name="batchnorm41_1_32_")
        add42_1_ = batchnorm41_1_1_ + batchnorm41_1_2_ + batchnorm41_1_3_ + batchnorm41_1_4_ + batchnorm41_1_5_ + batchnorm41_1_6_ + batchnorm41_1_7_ + batchnorm41_1_8_ + batchnorm41_1_9_ + batchnorm41_1_10_ + batchnorm41_1_11_ + batchnorm41_1_12_ + batchnorm41_1_13_ + batchnorm41_1_14_ + batchnorm41_1_15_ + batchnorm41_1_16_ + batchnorm41_1_17_ + batchnorm41_1_18_ + batchnorm41_1_19_ + batchnorm41_1_20_ + batchnorm41_1_21_ + batchnorm41_1_22_ + batchnorm41_1_23_ + batchnorm41_1_24_ + batchnorm41_1_25_ + batchnorm41_1_26_ + batchnorm41_1_27_ + batchnorm41_1_28_ + batchnorm41_1_29_ + batchnorm41_1_30_ + batchnorm41_1_31_ + batchnorm41_1_32_
        # add42_1_, output shape: {[512,28,28]}

        add43_ = add42_1_ + relu37_
        # add43_, output shape: {[512,28,28]}

        relu43_ = mx.symbol.Activation(data=add43_,
            act_type='relu',
            name="relu43_")

        conv45_1_1_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_1_")
        # conv45_1_1_, output shape: {[16,28,28]}

        batchnorm45_1_1_ = mx.symbol.BatchNorm(data=conv45_1_1_,
            fix_gamma=True,
            name="batchnorm45_1_1_")
        relu45_1_1_ = mx.symbol.Activation(data=batchnorm45_1_1_,
            act_type='relu',
            name="relu45_1_1_")

        conv46_1_1_ = mx.symbol.pad(data=relu45_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_1_ = mx.symbol.Convolution(data=conv46_1_1_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_1_")
        # conv46_1_1_, output shape: {[16,14,14]}

        batchnorm46_1_1_ = mx.symbol.BatchNorm(data=conv46_1_1_,
            fix_gamma=True,
            name="batchnorm46_1_1_")
        relu46_1_1_ = mx.symbol.Activation(data=batchnorm46_1_1_,
            act_type='relu',
            name="relu46_1_1_")

        conv47_1_1_ = mx.symbol.Convolution(data=relu46_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_1_")
        # conv47_1_1_, output shape: {[1024,14,14]}

        batchnorm47_1_1_ = mx.symbol.BatchNorm(data=conv47_1_1_,
            fix_gamma=True,
            name="batchnorm47_1_1_")
        conv45_1_2_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_2_")
        # conv45_1_2_, output shape: {[16,28,28]}

        batchnorm45_1_2_ = mx.symbol.BatchNorm(data=conv45_1_2_,
            fix_gamma=True,
            name="batchnorm45_1_2_")
        relu45_1_2_ = mx.symbol.Activation(data=batchnorm45_1_2_,
            act_type='relu',
            name="relu45_1_2_")

        conv46_1_2_ = mx.symbol.pad(data=relu45_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_2_ = mx.symbol.Convolution(data=conv46_1_2_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_2_")
        # conv46_1_2_, output shape: {[16,14,14]}

        batchnorm46_1_2_ = mx.symbol.BatchNorm(data=conv46_1_2_,
            fix_gamma=True,
            name="batchnorm46_1_2_")
        relu46_1_2_ = mx.symbol.Activation(data=batchnorm46_1_2_,
            act_type='relu',
            name="relu46_1_2_")

        conv47_1_2_ = mx.symbol.Convolution(data=relu46_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_2_")
        # conv47_1_2_, output shape: {[1024,14,14]}

        batchnorm47_1_2_ = mx.symbol.BatchNorm(data=conv47_1_2_,
            fix_gamma=True,
            name="batchnorm47_1_2_")
        conv45_1_3_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_3_")
        # conv45_1_3_, output shape: {[16,28,28]}

        batchnorm45_1_3_ = mx.symbol.BatchNorm(data=conv45_1_3_,
            fix_gamma=True,
            name="batchnorm45_1_3_")
        relu45_1_3_ = mx.symbol.Activation(data=batchnorm45_1_3_,
            act_type='relu',
            name="relu45_1_3_")

        conv46_1_3_ = mx.symbol.pad(data=relu45_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_3_ = mx.symbol.Convolution(data=conv46_1_3_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_3_")
        # conv46_1_3_, output shape: {[16,14,14]}

        batchnorm46_1_3_ = mx.symbol.BatchNorm(data=conv46_1_3_,
            fix_gamma=True,
            name="batchnorm46_1_3_")
        relu46_1_3_ = mx.symbol.Activation(data=batchnorm46_1_3_,
            act_type='relu',
            name="relu46_1_3_")

        conv47_1_3_ = mx.symbol.Convolution(data=relu46_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_3_")
        # conv47_1_3_, output shape: {[1024,14,14]}

        batchnorm47_1_3_ = mx.symbol.BatchNorm(data=conv47_1_3_,
            fix_gamma=True,
            name="batchnorm47_1_3_")
        conv45_1_4_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_4_")
        # conv45_1_4_, output shape: {[16,28,28]}

        batchnorm45_1_4_ = mx.symbol.BatchNorm(data=conv45_1_4_,
            fix_gamma=True,
            name="batchnorm45_1_4_")
        relu45_1_4_ = mx.symbol.Activation(data=batchnorm45_1_4_,
            act_type='relu',
            name="relu45_1_4_")

        conv46_1_4_ = mx.symbol.pad(data=relu45_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_4_ = mx.symbol.Convolution(data=conv46_1_4_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_4_")
        # conv46_1_4_, output shape: {[16,14,14]}

        batchnorm46_1_4_ = mx.symbol.BatchNorm(data=conv46_1_4_,
            fix_gamma=True,
            name="batchnorm46_1_4_")
        relu46_1_4_ = mx.symbol.Activation(data=batchnorm46_1_4_,
            act_type='relu',
            name="relu46_1_4_")

        conv47_1_4_ = mx.symbol.Convolution(data=relu46_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_4_")
        # conv47_1_4_, output shape: {[1024,14,14]}

        batchnorm47_1_4_ = mx.symbol.BatchNorm(data=conv47_1_4_,
            fix_gamma=True,
            name="batchnorm47_1_4_")
        conv45_1_5_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_5_")
        # conv45_1_5_, output shape: {[16,28,28]}

        batchnorm45_1_5_ = mx.symbol.BatchNorm(data=conv45_1_5_,
            fix_gamma=True,
            name="batchnorm45_1_5_")
        relu45_1_5_ = mx.symbol.Activation(data=batchnorm45_1_5_,
            act_type='relu',
            name="relu45_1_5_")

        conv46_1_5_ = mx.symbol.pad(data=relu45_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_5_ = mx.symbol.Convolution(data=conv46_1_5_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_5_")
        # conv46_1_5_, output shape: {[16,14,14]}

        batchnorm46_1_5_ = mx.symbol.BatchNorm(data=conv46_1_5_,
            fix_gamma=True,
            name="batchnorm46_1_5_")
        relu46_1_5_ = mx.symbol.Activation(data=batchnorm46_1_5_,
            act_type='relu',
            name="relu46_1_5_")

        conv47_1_5_ = mx.symbol.Convolution(data=relu46_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_5_")
        # conv47_1_5_, output shape: {[1024,14,14]}

        batchnorm47_1_5_ = mx.symbol.BatchNorm(data=conv47_1_5_,
            fix_gamma=True,
            name="batchnorm47_1_5_")
        conv45_1_6_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_6_")
        # conv45_1_6_, output shape: {[16,28,28]}

        batchnorm45_1_6_ = mx.symbol.BatchNorm(data=conv45_1_6_,
            fix_gamma=True,
            name="batchnorm45_1_6_")
        relu45_1_6_ = mx.symbol.Activation(data=batchnorm45_1_6_,
            act_type='relu',
            name="relu45_1_6_")

        conv46_1_6_ = mx.symbol.pad(data=relu45_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_6_ = mx.symbol.Convolution(data=conv46_1_6_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_6_")
        # conv46_1_6_, output shape: {[16,14,14]}

        batchnorm46_1_6_ = mx.symbol.BatchNorm(data=conv46_1_6_,
            fix_gamma=True,
            name="batchnorm46_1_6_")
        relu46_1_6_ = mx.symbol.Activation(data=batchnorm46_1_6_,
            act_type='relu',
            name="relu46_1_6_")

        conv47_1_6_ = mx.symbol.Convolution(data=relu46_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_6_")
        # conv47_1_6_, output shape: {[1024,14,14]}

        batchnorm47_1_6_ = mx.symbol.BatchNorm(data=conv47_1_6_,
            fix_gamma=True,
            name="batchnorm47_1_6_")
        conv45_1_7_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_7_")
        # conv45_1_7_, output shape: {[16,28,28]}

        batchnorm45_1_7_ = mx.symbol.BatchNorm(data=conv45_1_7_,
            fix_gamma=True,
            name="batchnorm45_1_7_")
        relu45_1_7_ = mx.symbol.Activation(data=batchnorm45_1_7_,
            act_type='relu',
            name="relu45_1_7_")

        conv46_1_7_ = mx.symbol.pad(data=relu45_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_7_ = mx.symbol.Convolution(data=conv46_1_7_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_7_")
        # conv46_1_7_, output shape: {[16,14,14]}

        batchnorm46_1_7_ = mx.symbol.BatchNorm(data=conv46_1_7_,
            fix_gamma=True,
            name="batchnorm46_1_7_")
        relu46_1_7_ = mx.symbol.Activation(data=batchnorm46_1_7_,
            act_type='relu',
            name="relu46_1_7_")

        conv47_1_7_ = mx.symbol.Convolution(data=relu46_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_7_")
        # conv47_1_7_, output shape: {[1024,14,14]}

        batchnorm47_1_7_ = mx.symbol.BatchNorm(data=conv47_1_7_,
            fix_gamma=True,
            name="batchnorm47_1_7_")
        conv45_1_8_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_8_")
        # conv45_1_8_, output shape: {[16,28,28]}

        batchnorm45_1_8_ = mx.symbol.BatchNorm(data=conv45_1_8_,
            fix_gamma=True,
            name="batchnorm45_1_8_")
        relu45_1_8_ = mx.symbol.Activation(data=batchnorm45_1_8_,
            act_type='relu',
            name="relu45_1_8_")

        conv46_1_8_ = mx.symbol.pad(data=relu45_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_8_ = mx.symbol.Convolution(data=conv46_1_8_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_8_")
        # conv46_1_8_, output shape: {[16,14,14]}

        batchnorm46_1_8_ = mx.symbol.BatchNorm(data=conv46_1_8_,
            fix_gamma=True,
            name="batchnorm46_1_8_")
        relu46_1_8_ = mx.symbol.Activation(data=batchnorm46_1_8_,
            act_type='relu',
            name="relu46_1_8_")

        conv47_1_8_ = mx.symbol.Convolution(data=relu46_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_8_")
        # conv47_1_8_, output shape: {[1024,14,14]}

        batchnorm47_1_8_ = mx.symbol.BatchNorm(data=conv47_1_8_,
            fix_gamma=True,
            name="batchnorm47_1_8_")
        conv45_1_9_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_9_")
        # conv45_1_9_, output shape: {[16,28,28]}

        batchnorm45_1_9_ = mx.symbol.BatchNorm(data=conv45_1_9_,
            fix_gamma=True,
            name="batchnorm45_1_9_")
        relu45_1_9_ = mx.symbol.Activation(data=batchnorm45_1_9_,
            act_type='relu',
            name="relu45_1_9_")

        conv46_1_9_ = mx.symbol.pad(data=relu45_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_9_ = mx.symbol.Convolution(data=conv46_1_9_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_9_")
        # conv46_1_9_, output shape: {[16,14,14]}

        batchnorm46_1_9_ = mx.symbol.BatchNorm(data=conv46_1_9_,
            fix_gamma=True,
            name="batchnorm46_1_9_")
        relu46_1_9_ = mx.symbol.Activation(data=batchnorm46_1_9_,
            act_type='relu',
            name="relu46_1_9_")

        conv47_1_9_ = mx.symbol.Convolution(data=relu46_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_9_")
        # conv47_1_9_, output shape: {[1024,14,14]}

        batchnorm47_1_9_ = mx.symbol.BatchNorm(data=conv47_1_9_,
            fix_gamma=True,
            name="batchnorm47_1_9_")
        conv45_1_10_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_10_")
        # conv45_1_10_, output shape: {[16,28,28]}

        batchnorm45_1_10_ = mx.symbol.BatchNorm(data=conv45_1_10_,
            fix_gamma=True,
            name="batchnorm45_1_10_")
        relu45_1_10_ = mx.symbol.Activation(data=batchnorm45_1_10_,
            act_type='relu',
            name="relu45_1_10_")

        conv46_1_10_ = mx.symbol.pad(data=relu45_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_10_ = mx.symbol.Convolution(data=conv46_1_10_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_10_")
        # conv46_1_10_, output shape: {[16,14,14]}

        batchnorm46_1_10_ = mx.symbol.BatchNorm(data=conv46_1_10_,
            fix_gamma=True,
            name="batchnorm46_1_10_")
        relu46_1_10_ = mx.symbol.Activation(data=batchnorm46_1_10_,
            act_type='relu',
            name="relu46_1_10_")

        conv47_1_10_ = mx.symbol.Convolution(data=relu46_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_10_")
        # conv47_1_10_, output shape: {[1024,14,14]}

        batchnorm47_1_10_ = mx.symbol.BatchNorm(data=conv47_1_10_,
            fix_gamma=True,
            name="batchnorm47_1_10_")
        conv45_1_11_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_11_")
        # conv45_1_11_, output shape: {[16,28,28]}

        batchnorm45_1_11_ = mx.symbol.BatchNorm(data=conv45_1_11_,
            fix_gamma=True,
            name="batchnorm45_1_11_")
        relu45_1_11_ = mx.symbol.Activation(data=batchnorm45_1_11_,
            act_type='relu',
            name="relu45_1_11_")

        conv46_1_11_ = mx.symbol.pad(data=relu45_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_11_ = mx.symbol.Convolution(data=conv46_1_11_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_11_")
        # conv46_1_11_, output shape: {[16,14,14]}

        batchnorm46_1_11_ = mx.symbol.BatchNorm(data=conv46_1_11_,
            fix_gamma=True,
            name="batchnorm46_1_11_")
        relu46_1_11_ = mx.symbol.Activation(data=batchnorm46_1_11_,
            act_type='relu',
            name="relu46_1_11_")

        conv47_1_11_ = mx.symbol.Convolution(data=relu46_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_11_")
        # conv47_1_11_, output shape: {[1024,14,14]}

        batchnorm47_1_11_ = mx.symbol.BatchNorm(data=conv47_1_11_,
            fix_gamma=True,
            name="batchnorm47_1_11_")
        conv45_1_12_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_12_")
        # conv45_1_12_, output shape: {[16,28,28]}

        batchnorm45_1_12_ = mx.symbol.BatchNorm(data=conv45_1_12_,
            fix_gamma=True,
            name="batchnorm45_1_12_")
        relu45_1_12_ = mx.symbol.Activation(data=batchnorm45_1_12_,
            act_type='relu',
            name="relu45_1_12_")

        conv46_1_12_ = mx.symbol.pad(data=relu45_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_12_ = mx.symbol.Convolution(data=conv46_1_12_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_12_")
        # conv46_1_12_, output shape: {[16,14,14]}

        batchnorm46_1_12_ = mx.symbol.BatchNorm(data=conv46_1_12_,
            fix_gamma=True,
            name="batchnorm46_1_12_")
        relu46_1_12_ = mx.symbol.Activation(data=batchnorm46_1_12_,
            act_type='relu',
            name="relu46_1_12_")

        conv47_1_12_ = mx.symbol.Convolution(data=relu46_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_12_")
        # conv47_1_12_, output shape: {[1024,14,14]}

        batchnorm47_1_12_ = mx.symbol.BatchNorm(data=conv47_1_12_,
            fix_gamma=True,
            name="batchnorm47_1_12_")
        conv45_1_13_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_13_")
        # conv45_1_13_, output shape: {[16,28,28]}

        batchnorm45_1_13_ = mx.symbol.BatchNorm(data=conv45_1_13_,
            fix_gamma=True,
            name="batchnorm45_1_13_")
        relu45_1_13_ = mx.symbol.Activation(data=batchnorm45_1_13_,
            act_type='relu',
            name="relu45_1_13_")

        conv46_1_13_ = mx.symbol.pad(data=relu45_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_13_ = mx.symbol.Convolution(data=conv46_1_13_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_13_")
        # conv46_1_13_, output shape: {[16,14,14]}

        batchnorm46_1_13_ = mx.symbol.BatchNorm(data=conv46_1_13_,
            fix_gamma=True,
            name="batchnorm46_1_13_")
        relu46_1_13_ = mx.symbol.Activation(data=batchnorm46_1_13_,
            act_type='relu',
            name="relu46_1_13_")

        conv47_1_13_ = mx.symbol.Convolution(data=relu46_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_13_")
        # conv47_1_13_, output shape: {[1024,14,14]}

        batchnorm47_1_13_ = mx.symbol.BatchNorm(data=conv47_1_13_,
            fix_gamma=True,
            name="batchnorm47_1_13_")
        conv45_1_14_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_14_")
        # conv45_1_14_, output shape: {[16,28,28]}

        batchnorm45_1_14_ = mx.symbol.BatchNorm(data=conv45_1_14_,
            fix_gamma=True,
            name="batchnorm45_1_14_")
        relu45_1_14_ = mx.symbol.Activation(data=batchnorm45_1_14_,
            act_type='relu',
            name="relu45_1_14_")

        conv46_1_14_ = mx.symbol.pad(data=relu45_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_14_ = mx.symbol.Convolution(data=conv46_1_14_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_14_")
        # conv46_1_14_, output shape: {[16,14,14]}

        batchnorm46_1_14_ = mx.symbol.BatchNorm(data=conv46_1_14_,
            fix_gamma=True,
            name="batchnorm46_1_14_")
        relu46_1_14_ = mx.symbol.Activation(data=batchnorm46_1_14_,
            act_type='relu',
            name="relu46_1_14_")

        conv47_1_14_ = mx.symbol.Convolution(data=relu46_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_14_")
        # conv47_1_14_, output shape: {[1024,14,14]}

        batchnorm47_1_14_ = mx.symbol.BatchNorm(data=conv47_1_14_,
            fix_gamma=True,
            name="batchnorm47_1_14_")
        conv45_1_15_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_15_")
        # conv45_1_15_, output shape: {[16,28,28]}

        batchnorm45_1_15_ = mx.symbol.BatchNorm(data=conv45_1_15_,
            fix_gamma=True,
            name="batchnorm45_1_15_")
        relu45_1_15_ = mx.symbol.Activation(data=batchnorm45_1_15_,
            act_type='relu',
            name="relu45_1_15_")

        conv46_1_15_ = mx.symbol.pad(data=relu45_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_15_ = mx.symbol.Convolution(data=conv46_1_15_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_15_")
        # conv46_1_15_, output shape: {[16,14,14]}

        batchnorm46_1_15_ = mx.symbol.BatchNorm(data=conv46_1_15_,
            fix_gamma=True,
            name="batchnorm46_1_15_")
        relu46_1_15_ = mx.symbol.Activation(data=batchnorm46_1_15_,
            act_type='relu',
            name="relu46_1_15_")

        conv47_1_15_ = mx.symbol.Convolution(data=relu46_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_15_")
        # conv47_1_15_, output shape: {[1024,14,14]}

        batchnorm47_1_15_ = mx.symbol.BatchNorm(data=conv47_1_15_,
            fix_gamma=True,
            name="batchnorm47_1_15_")
        conv45_1_16_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_16_")
        # conv45_1_16_, output shape: {[16,28,28]}

        batchnorm45_1_16_ = mx.symbol.BatchNorm(data=conv45_1_16_,
            fix_gamma=True,
            name="batchnorm45_1_16_")
        relu45_1_16_ = mx.symbol.Activation(data=batchnorm45_1_16_,
            act_type='relu',
            name="relu45_1_16_")

        conv46_1_16_ = mx.symbol.pad(data=relu45_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_16_ = mx.symbol.Convolution(data=conv46_1_16_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_16_")
        # conv46_1_16_, output shape: {[16,14,14]}

        batchnorm46_1_16_ = mx.symbol.BatchNorm(data=conv46_1_16_,
            fix_gamma=True,
            name="batchnorm46_1_16_")
        relu46_1_16_ = mx.symbol.Activation(data=batchnorm46_1_16_,
            act_type='relu',
            name="relu46_1_16_")

        conv47_1_16_ = mx.symbol.Convolution(data=relu46_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_16_")
        # conv47_1_16_, output shape: {[1024,14,14]}

        batchnorm47_1_16_ = mx.symbol.BatchNorm(data=conv47_1_16_,
            fix_gamma=True,
            name="batchnorm47_1_16_")
        conv45_1_17_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_17_")
        # conv45_1_17_, output shape: {[16,28,28]}

        batchnorm45_1_17_ = mx.symbol.BatchNorm(data=conv45_1_17_,
            fix_gamma=True,
            name="batchnorm45_1_17_")
        relu45_1_17_ = mx.symbol.Activation(data=batchnorm45_1_17_,
            act_type='relu',
            name="relu45_1_17_")

        conv46_1_17_ = mx.symbol.pad(data=relu45_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_17_ = mx.symbol.Convolution(data=conv46_1_17_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_17_")
        # conv46_1_17_, output shape: {[16,14,14]}

        batchnorm46_1_17_ = mx.symbol.BatchNorm(data=conv46_1_17_,
            fix_gamma=True,
            name="batchnorm46_1_17_")
        relu46_1_17_ = mx.symbol.Activation(data=batchnorm46_1_17_,
            act_type='relu',
            name="relu46_1_17_")

        conv47_1_17_ = mx.symbol.Convolution(data=relu46_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_17_")
        # conv47_1_17_, output shape: {[1024,14,14]}

        batchnorm47_1_17_ = mx.symbol.BatchNorm(data=conv47_1_17_,
            fix_gamma=True,
            name="batchnorm47_1_17_")
        conv45_1_18_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_18_")
        # conv45_1_18_, output shape: {[16,28,28]}

        batchnorm45_1_18_ = mx.symbol.BatchNorm(data=conv45_1_18_,
            fix_gamma=True,
            name="batchnorm45_1_18_")
        relu45_1_18_ = mx.symbol.Activation(data=batchnorm45_1_18_,
            act_type='relu',
            name="relu45_1_18_")

        conv46_1_18_ = mx.symbol.pad(data=relu45_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_18_ = mx.symbol.Convolution(data=conv46_1_18_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_18_")
        # conv46_1_18_, output shape: {[16,14,14]}

        batchnorm46_1_18_ = mx.symbol.BatchNorm(data=conv46_1_18_,
            fix_gamma=True,
            name="batchnorm46_1_18_")
        relu46_1_18_ = mx.symbol.Activation(data=batchnorm46_1_18_,
            act_type='relu',
            name="relu46_1_18_")

        conv47_1_18_ = mx.symbol.Convolution(data=relu46_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_18_")
        # conv47_1_18_, output shape: {[1024,14,14]}

        batchnorm47_1_18_ = mx.symbol.BatchNorm(data=conv47_1_18_,
            fix_gamma=True,
            name="batchnorm47_1_18_")
        conv45_1_19_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_19_")
        # conv45_1_19_, output shape: {[16,28,28]}

        batchnorm45_1_19_ = mx.symbol.BatchNorm(data=conv45_1_19_,
            fix_gamma=True,
            name="batchnorm45_1_19_")
        relu45_1_19_ = mx.symbol.Activation(data=batchnorm45_1_19_,
            act_type='relu',
            name="relu45_1_19_")

        conv46_1_19_ = mx.symbol.pad(data=relu45_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_19_ = mx.symbol.Convolution(data=conv46_1_19_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_19_")
        # conv46_1_19_, output shape: {[16,14,14]}

        batchnorm46_1_19_ = mx.symbol.BatchNorm(data=conv46_1_19_,
            fix_gamma=True,
            name="batchnorm46_1_19_")
        relu46_1_19_ = mx.symbol.Activation(data=batchnorm46_1_19_,
            act_type='relu',
            name="relu46_1_19_")

        conv47_1_19_ = mx.symbol.Convolution(data=relu46_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_19_")
        # conv47_1_19_, output shape: {[1024,14,14]}

        batchnorm47_1_19_ = mx.symbol.BatchNorm(data=conv47_1_19_,
            fix_gamma=True,
            name="batchnorm47_1_19_")
        conv45_1_20_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_20_")
        # conv45_1_20_, output shape: {[16,28,28]}

        batchnorm45_1_20_ = mx.symbol.BatchNorm(data=conv45_1_20_,
            fix_gamma=True,
            name="batchnorm45_1_20_")
        relu45_1_20_ = mx.symbol.Activation(data=batchnorm45_1_20_,
            act_type='relu',
            name="relu45_1_20_")

        conv46_1_20_ = mx.symbol.pad(data=relu45_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_20_ = mx.symbol.Convolution(data=conv46_1_20_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_20_")
        # conv46_1_20_, output shape: {[16,14,14]}

        batchnorm46_1_20_ = mx.symbol.BatchNorm(data=conv46_1_20_,
            fix_gamma=True,
            name="batchnorm46_1_20_")
        relu46_1_20_ = mx.symbol.Activation(data=batchnorm46_1_20_,
            act_type='relu',
            name="relu46_1_20_")

        conv47_1_20_ = mx.symbol.Convolution(data=relu46_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_20_")
        # conv47_1_20_, output shape: {[1024,14,14]}

        batchnorm47_1_20_ = mx.symbol.BatchNorm(data=conv47_1_20_,
            fix_gamma=True,
            name="batchnorm47_1_20_")
        conv45_1_21_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_21_")
        # conv45_1_21_, output shape: {[16,28,28]}

        batchnorm45_1_21_ = mx.symbol.BatchNorm(data=conv45_1_21_,
            fix_gamma=True,
            name="batchnorm45_1_21_")
        relu45_1_21_ = mx.symbol.Activation(data=batchnorm45_1_21_,
            act_type='relu',
            name="relu45_1_21_")

        conv46_1_21_ = mx.symbol.pad(data=relu45_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_21_ = mx.symbol.Convolution(data=conv46_1_21_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_21_")
        # conv46_1_21_, output shape: {[16,14,14]}

        batchnorm46_1_21_ = mx.symbol.BatchNorm(data=conv46_1_21_,
            fix_gamma=True,
            name="batchnorm46_1_21_")
        relu46_1_21_ = mx.symbol.Activation(data=batchnorm46_1_21_,
            act_type='relu',
            name="relu46_1_21_")

        conv47_1_21_ = mx.symbol.Convolution(data=relu46_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_21_")
        # conv47_1_21_, output shape: {[1024,14,14]}

        batchnorm47_1_21_ = mx.symbol.BatchNorm(data=conv47_1_21_,
            fix_gamma=True,
            name="batchnorm47_1_21_")
        conv45_1_22_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_22_")
        # conv45_1_22_, output shape: {[16,28,28]}

        batchnorm45_1_22_ = mx.symbol.BatchNorm(data=conv45_1_22_,
            fix_gamma=True,
            name="batchnorm45_1_22_")
        relu45_1_22_ = mx.symbol.Activation(data=batchnorm45_1_22_,
            act_type='relu',
            name="relu45_1_22_")

        conv46_1_22_ = mx.symbol.pad(data=relu45_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_22_ = mx.symbol.Convolution(data=conv46_1_22_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_22_")
        # conv46_1_22_, output shape: {[16,14,14]}

        batchnorm46_1_22_ = mx.symbol.BatchNorm(data=conv46_1_22_,
            fix_gamma=True,
            name="batchnorm46_1_22_")
        relu46_1_22_ = mx.symbol.Activation(data=batchnorm46_1_22_,
            act_type='relu',
            name="relu46_1_22_")

        conv47_1_22_ = mx.symbol.Convolution(data=relu46_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_22_")
        # conv47_1_22_, output shape: {[1024,14,14]}

        batchnorm47_1_22_ = mx.symbol.BatchNorm(data=conv47_1_22_,
            fix_gamma=True,
            name="batchnorm47_1_22_")
        conv45_1_23_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_23_")
        # conv45_1_23_, output shape: {[16,28,28]}

        batchnorm45_1_23_ = mx.symbol.BatchNorm(data=conv45_1_23_,
            fix_gamma=True,
            name="batchnorm45_1_23_")
        relu45_1_23_ = mx.symbol.Activation(data=batchnorm45_1_23_,
            act_type='relu',
            name="relu45_1_23_")

        conv46_1_23_ = mx.symbol.pad(data=relu45_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_23_ = mx.symbol.Convolution(data=conv46_1_23_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_23_")
        # conv46_1_23_, output shape: {[16,14,14]}

        batchnorm46_1_23_ = mx.symbol.BatchNorm(data=conv46_1_23_,
            fix_gamma=True,
            name="batchnorm46_1_23_")
        relu46_1_23_ = mx.symbol.Activation(data=batchnorm46_1_23_,
            act_type='relu',
            name="relu46_1_23_")

        conv47_1_23_ = mx.symbol.Convolution(data=relu46_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_23_")
        # conv47_1_23_, output shape: {[1024,14,14]}

        batchnorm47_1_23_ = mx.symbol.BatchNorm(data=conv47_1_23_,
            fix_gamma=True,
            name="batchnorm47_1_23_")
        conv45_1_24_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_24_")
        # conv45_1_24_, output shape: {[16,28,28]}

        batchnorm45_1_24_ = mx.symbol.BatchNorm(data=conv45_1_24_,
            fix_gamma=True,
            name="batchnorm45_1_24_")
        relu45_1_24_ = mx.symbol.Activation(data=batchnorm45_1_24_,
            act_type='relu',
            name="relu45_1_24_")

        conv46_1_24_ = mx.symbol.pad(data=relu45_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_24_ = mx.symbol.Convolution(data=conv46_1_24_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_24_")
        # conv46_1_24_, output shape: {[16,14,14]}

        batchnorm46_1_24_ = mx.symbol.BatchNorm(data=conv46_1_24_,
            fix_gamma=True,
            name="batchnorm46_1_24_")
        relu46_1_24_ = mx.symbol.Activation(data=batchnorm46_1_24_,
            act_type='relu',
            name="relu46_1_24_")

        conv47_1_24_ = mx.symbol.Convolution(data=relu46_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_24_")
        # conv47_1_24_, output shape: {[1024,14,14]}

        batchnorm47_1_24_ = mx.symbol.BatchNorm(data=conv47_1_24_,
            fix_gamma=True,
            name="batchnorm47_1_24_")
        conv45_1_25_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_25_")
        # conv45_1_25_, output shape: {[16,28,28]}

        batchnorm45_1_25_ = mx.symbol.BatchNorm(data=conv45_1_25_,
            fix_gamma=True,
            name="batchnorm45_1_25_")
        relu45_1_25_ = mx.symbol.Activation(data=batchnorm45_1_25_,
            act_type='relu',
            name="relu45_1_25_")

        conv46_1_25_ = mx.symbol.pad(data=relu45_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_25_ = mx.symbol.Convolution(data=conv46_1_25_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_25_")
        # conv46_1_25_, output shape: {[16,14,14]}

        batchnorm46_1_25_ = mx.symbol.BatchNorm(data=conv46_1_25_,
            fix_gamma=True,
            name="batchnorm46_1_25_")
        relu46_1_25_ = mx.symbol.Activation(data=batchnorm46_1_25_,
            act_type='relu',
            name="relu46_1_25_")

        conv47_1_25_ = mx.symbol.Convolution(data=relu46_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_25_")
        # conv47_1_25_, output shape: {[1024,14,14]}

        batchnorm47_1_25_ = mx.symbol.BatchNorm(data=conv47_1_25_,
            fix_gamma=True,
            name="batchnorm47_1_25_")
        conv45_1_26_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_26_")
        # conv45_1_26_, output shape: {[16,28,28]}

        batchnorm45_1_26_ = mx.symbol.BatchNorm(data=conv45_1_26_,
            fix_gamma=True,
            name="batchnorm45_1_26_")
        relu45_1_26_ = mx.symbol.Activation(data=batchnorm45_1_26_,
            act_type='relu',
            name="relu45_1_26_")

        conv46_1_26_ = mx.symbol.pad(data=relu45_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_26_ = mx.symbol.Convolution(data=conv46_1_26_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_26_")
        # conv46_1_26_, output shape: {[16,14,14]}

        batchnorm46_1_26_ = mx.symbol.BatchNorm(data=conv46_1_26_,
            fix_gamma=True,
            name="batchnorm46_1_26_")
        relu46_1_26_ = mx.symbol.Activation(data=batchnorm46_1_26_,
            act_type='relu',
            name="relu46_1_26_")

        conv47_1_26_ = mx.symbol.Convolution(data=relu46_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_26_")
        # conv47_1_26_, output shape: {[1024,14,14]}

        batchnorm47_1_26_ = mx.symbol.BatchNorm(data=conv47_1_26_,
            fix_gamma=True,
            name="batchnorm47_1_26_")
        conv45_1_27_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_27_")
        # conv45_1_27_, output shape: {[16,28,28]}

        batchnorm45_1_27_ = mx.symbol.BatchNorm(data=conv45_1_27_,
            fix_gamma=True,
            name="batchnorm45_1_27_")
        relu45_1_27_ = mx.symbol.Activation(data=batchnorm45_1_27_,
            act_type='relu',
            name="relu45_1_27_")

        conv46_1_27_ = mx.symbol.pad(data=relu45_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_27_ = mx.symbol.Convolution(data=conv46_1_27_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_27_")
        # conv46_1_27_, output shape: {[16,14,14]}

        batchnorm46_1_27_ = mx.symbol.BatchNorm(data=conv46_1_27_,
            fix_gamma=True,
            name="batchnorm46_1_27_")
        relu46_1_27_ = mx.symbol.Activation(data=batchnorm46_1_27_,
            act_type='relu',
            name="relu46_1_27_")

        conv47_1_27_ = mx.symbol.Convolution(data=relu46_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_27_")
        # conv47_1_27_, output shape: {[1024,14,14]}

        batchnorm47_1_27_ = mx.symbol.BatchNorm(data=conv47_1_27_,
            fix_gamma=True,
            name="batchnorm47_1_27_")
        conv45_1_28_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_28_")
        # conv45_1_28_, output shape: {[16,28,28]}

        batchnorm45_1_28_ = mx.symbol.BatchNorm(data=conv45_1_28_,
            fix_gamma=True,
            name="batchnorm45_1_28_")
        relu45_1_28_ = mx.symbol.Activation(data=batchnorm45_1_28_,
            act_type='relu',
            name="relu45_1_28_")

        conv46_1_28_ = mx.symbol.pad(data=relu45_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_28_ = mx.symbol.Convolution(data=conv46_1_28_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_28_")
        # conv46_1_28_, output shape: {[16,14,14]}

        batchnorm46_1_28_ = mx.symbol.BatchNorm(data=conv46_1_28_,
            fix_gamma=True,
            name="batchnorm46_1_28_")
        relu46_1_28_ = mx.symbol.Activation(data=batchnorm46_1_28_,
            act_type='relu',
            name="relu46_1_28_")

        conv47_1_28_ = mx.symbol.Convolution(data=relu46_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_28_")
        # conv47_1_28_, output shape: {[1024,14,14]}

        batchnorm47_1_28_ = mx.symbol.BatchNorm(data=conv47_1_28_,
            fix_gamma=True,
            name="batchnorm47_1_28_")
        conv45_1_29_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_29_")
        # conv45_1_29_, output shape: {[16,28,28]}

        batchnorm45_1_29_ = mx.symbol.BatchNorm(data=conv45_1_29_,
            fix_gamma=True,
            name="batchnorm45_1_29_")
        relu45_1_29_ = mx.symbol.Activation(data=batchnorm45_1_29_,
            act_type='relu',
            name="relu45_1_29_")

        conv46_1_29_ = mx.symbol.pad(data=relu45_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_29_ = mx.symbol.Convolution(data=conv46_1_29_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_29_")
        # conv46_1_29_, output shape: {[16,14,14]}

        batchnorm46_1_29_ = mx.symbol.BatchNorm(data=conv46_1_29_,
            fix_gamma=True,
            name="batchnorm46_1_29_")
        relu46_1_29_ = mx.symbol.Activation(data=batchnorm46_1_29_,
            act_type='relu',
            name="relu46_1_29_")

        conv47_1_29_ = mx.symbol.Convolution(data=relu46_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_29_")
        # conv47_1_29_, output shape: {[1024,14,14]}

        batchnorm47_1_29_ = mx.symbol.BatchNorm(data=conv47_1_29_,
            fix_gamma=True,
            name="batchnorm47_1_29_")
        conv45_1_30_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_30_")
        # conv45_1_30_, output shape: {[16,28,28]}

        batchnorm45_1_30_ = mx.symbol.BatchNorm(data=conv45_1_30_,
            fix_gamma=True,
            name="batchnorm45_1_30_")
        relu45_1_30_ = mx.symbol.Activation(data=batchnorm45_1_30_,
            act_type='relu',
            name="relu45_1_30_")

        conv46_1_30_ = mx.symbol.pad(data=relu45_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_30_ = mx.symbol.Convolution(data=conv46_1_30_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_30_")
        # conv46_1_30_, output shape: {[16,14,14]}

        batchnorm46_1_30_ = mx.symbol.BatchNorm(data=conv46_1_30_,
            fix_gamma=True,
            name="batchnorm46_1_30_")
        relu46_1_30_ = mx.symbol.Activation(data=batchnorm46_1_30_,
            act_type='relu',
            name="relu46_1_30_")

        conv47_1_30_ = mx.symbol.Convolution(data=relu46_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_30_")
        # conv47_1_30_, output shape: {[1024,14,14]}

        batchnorm47_1_30_ = mx.symbol.BatchNorm(data=conv47_1_30_,
            fix_gamma=True,
            name="batchnorm47_1_30_")
        conv45_1_31_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_31_")
        # conv45_1_31_, output shape: {[16,28,28]}

        batchnorm45_1_31_ = mx.symbol.BatchNorm(data=conv45_1_31_,
            fix_gamma=True,
            name="batchnorm45_1_31_")
        relu45_1_31_ = mx.symbol.Activation(data=batchnorm45_1_31_,
            act_type='relu',
            name="relu45_1_31_")

        conv46_1_31_ = mx.symbol.pad(data=relu45_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_31_ = mx.symbol.Convolution(data=conv46_1_31_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_31_")
        # conv46_1_31_, output shape: {[16,14,14]}

        batchnorm46_1_31_ = mx.symbol.BatchNorm(data=conv46_1_31_,
            fix_gamma=True,
            name="batchnorm46_1_31_")
        relu46_1_31_ = mx.symbol.Activation(data=batchnorm46_1_31_,
            act_type='relu',
            name="relu46_1_31_")

        conv47_1_31_ = mx.symbol.Convolution(data=relu46_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_31_")
        # conv47_1_31_, output shape: {[1024,14,14]}

        batchnorm47_1_31_ = mx.symbol.BatchNorm(data=conv47_1_31_,
            fix_gamma=True,
            name="batchnorm47_1_31_")
        conv45_1_32_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv45_1_32_")
        # conv45_1_32_, output shape: {[16,28,28]}

        batchnorm45_1_32_ = mx.symbol.BatchNorm(data=conv45_1_32_,
            fix_gamma=True,
            name="batchnorm45_1_32_")
        relu45_1_32_ = mx.symbol.Activation(data=batchnorm45_1_32_,
            act_type='relu',
            name="relu45_1_32_")

        conv46_1_32_ = mx.symbol.pad(data=relu45_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv46_1_32_ = mx.symbol.Convolution(data=conv46_1_32_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=16,
            no_bias=False,
            name="conv46_1_32_")
        # conv46_1_32_, output shape: {[16,14,14]}

        batchnorm46_1_32_ = mx.symbol.BatchNorm(data=conv46_1_32_,
            fix_gamma=True,
            name="batchnorm46_1_32_")
        relu46_1_32_ = mx.symbol.Activation(data=batchnorm46_1_32_,
            act_type='relu',
            name="relu46_1_32_")

        conv47_1_32_ = mx.symbol.Convolution(data=relu46_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv47_1_32_")
        # conv47_1_32_, output shape: {[1024,14,14]}

        batchnorm47_1_32_ = mx.symbol.BatchNorm(data=conv47_1_32_,
            fix_gamma=True,
            name="batchnorm47_1_32_")
        add48_1_ = batchnorm47_1_1_ + batchnorm47_1_2_ + batchnorm47_1_3_ + batchnorm47_1_4_ + batchnorm47_1_5_ + batchnorm47_1_6_ + batchnorm47_1_7_ + batchnorm47_1_8_ + batchnorm47_1_9_ + batchnorm47_1_10_ + batchnorm47_1_11_ + batchnorm47_1_12_ + batchnorm47_1_13_ + batchnorm47_1_14_ + batchnorm47_1_15_ + batchnorm47_1_16_ + batchnorm47_1_17_ + batchnorm47_1_18_ + batchnorm47_1_19_ + batchnorm47_1_20_ + batchnorm47_1_21_ + batchnorm47_1_22_ + batchnorm47_1_23_ + batchnorm47_1_24_ + batchnorm47_1_25_ + batchnorm47_1_26_ + batchnorm47_1_27_ + batchnorm47_1_28_ + batchnorm47_1_29_ + batchnorm47_1_30_ + batchnorm47_1_31_ + batchnorm47_1_32_
        # add48_1_, output shape: {[1024,14,14]}

        conv44_2_ = mx.symbol.Convolution(data=relu43_,
            kernel=(1,1),
            stride=(2,2),
            num_filter=1024,
            no_bias=False,
            name="conv44_2_")
        # conv44_2_, output shape: {[1024,14,14]}

        batchnorm44_2_ = mx.symbol.BatchNorm(data=conv44_2_,
            fix_gamma=True,
            name="batchnorm44_2_")
        add49_ = add48_1_ + batchnorm44_2_
        # add49_, output shape: {[1024,14,14]}

        relu49_ = mx.symbol.Activation(data=add49_,
            act_type='relu',
            name="relu49_")

        conv51_1_1_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_1_")
        # conv51_1_1_, output shape: {[16,14,14]}

        batchnorm51_1_1_ = mx.symbol.BatchNorm(data=conv51_1_1_,
            fix_gamma=True,
            name="batchnorm51_1_1_")
        relu51_1_1_ = mx.symbol.Activation(data=batchnorm51_1_1_,
            act_type='relu',
            name="relu51_1_1_")

        conv52_1_1_ = mx.symbol.pad(data=relu51_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_1_ = mx.symbol.Convolution(data=conv52_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_1_")
        # conv52_1_1_, output shape: {[16,14,14]}

        batchnorm52_1_1_ = mx.symbol.BatchNorm(data=conv52_1_1_,
            fix_gamma=True,
            name="batchnorm52_1_1_")
        relu52_1_1_ = mx.symbol.Activation(data=batchnorm52_1_1_,
            act_type='relu',
            name="relu52_1_1_")

        conv53_1_1_ = mx.symbol.Convolution(data=relu52_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_1_")
        # conv53_1_1_, output shape: {[1024,14,14]}

        batchnorm53_1_1_ = mx.symbol.BatchNorm(data=conv53_1_1_,
            fix_gamma=True,
            name="batchnorm53_1_1_")
        conv51_1_2_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_2_")
        # conv51_1_2_, output shape: {[16,14,14]}

        batchnorm51_1_2_ = mx.symbol.BatchNorm(data=conv51_1_2_,
            fix_gamma=True,
            name="batchnorm51_1_2_")
        relu51_1_2_ = mx.symbol.Activation(data=batchnorm51_1_2_,
            act_type='relu',
            name="relu51_1_2_")

        conv52_1_2_ = mx.symbol.pad(data=relu51_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_2_ = mx.symbol.Convolution(data=conv52_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_2_")
        # conv52_1_2_, output shape: {[16,14,14]}

        batchnorm52_1_2_ = mx.symbol.BatchNorm(data=conv52_1_2_,
            fix_gamma=True,
            name="batchnorm52_1_2_")
        relu52_1_2_ = mx.symbol.Activation(data=batchnorm52_1_2_,
            act_type='relu',
            name="relu52_1_2_")

        conv53_1_2_ = mx.symbol.Convolution(data=relu52_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_2_")
        # conv53_1_2_, output shape: {[1024,14,14]}

        batchnorm53_1_2_ = mx.symbol.BatchNorm(data=conv53_1_2_,
            fix_gamma=True,
            name="batchnorm53_1_2_")
        conv51_1_3_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_3_")
        # conv51_1_3_, output shape: {[16,14,14]}

        batchnorm51_1_3_ = mx.symbol.BatchNorm(data=conv51_1_3_,
            fix_gamma=True,
            name="batchnorm51_1_3_")
        relu51_1_3_ = mx.symbol.Activation(data=batchnorm51_1_3_,
            act_type='relu',
            name="relu51_1_3_")

        conv52_1_3_ = mx.symbol.pad(data=relu51_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_3_ = mx.symbol.Convolution(data=conv52_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_3_")
        # conv52_1_3_, output shape: {[16,14,14]}

        batchnorm52_1_3_ = mx.symbol.BatchNorm(data=conv52_1_3_,
            fix_gamma=True,
            name="batchnorm52_1_3_")
        relu52_1_3_ = mx.symbol.Activation(data=batchnorm52_1_3_,
            act_type='relu',
            name="relu52_1_3_")

        conv53_1_3_ = mx.symbol.Convolution(data=relu52_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_3_")
        # conv53_1_3_, output shape: {[1024,14,14]}

        batchnorm53_1_3_ = mx.symbol.BatchNorm(data=conv53_1_3_,
            fix_gamma=True,
            name="batchnorm53_1_3_")
        conv51_1_4_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_4_")
        # conv51_1_4_, output shape: {[16,14,14]}

        batchnorm51_1_4_ = mx.symbol.BatchNorm(data=conv51_1_4_,
            fix_gamma=True,
            name="batchnorm51_1_4_")
        relu51_1_4_ = mx.symbol.Activation(data=batchnorm51_1_4_,
            act_type='relu',
            name="relu51_1_4_")

        conv52_1_4_ = mx.symbol.pad(data=relu51_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_4_ = mx.symbol.Convolution(data=conv52_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_4_")
        # conv52_1_4_, output shape: {[16,14,14]}

        batchnorm52_1_4_ = mx.symbol.BatchNorm(data=conv52_1_4_,
            fix_gamma=True,
            name="batchnorm52_1_4_")
        relu52_1_4_ = mx.symbol.Activation(data=batchnorm52_1_4_,
            act_type='relu',
            name="relu52_1_4_")

        conv53_1_4_ = mx.symbol.Convolution(data=relu52_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_4_")
        # conv53_1_4_, output shape: {[1024,14,14]}

        batchnorm53_1_4_ = mx.symbol.BatchNorm(data=conv53_1_4_,
            fix_gamma=True,
            name="batchnorm53_1_4_")
        conv51_1_5_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_5_")
        # conv51_1_5_, output shape: {[16,14,14]}

        batchnorm51_1_5_ = mx.symbol.BatchNorm(data=conv51_1_5_,
            fix_gamma=True,
            name="batchnorm51_1_5_")
        relu51_1_5_ = mx.symbol.Activation(data=batchnorm51_1_5_,
            act_type='relu',
            name="relu51_1_5_")

        conv52_1_5_ = mx.symbol.pad(data=relu51_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_5_ = mx.symbol.Convolution(data=conv52_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_5_")
        # conv52_1_5_, output shape: {[16,14,14]}

        batchnorm52_1_5_ = mx.symbol.BatchNorm(data=conv52_1_5_,
            fix_gamma=True,
            name="batchnorm52_1_5_")
        relu52_1_5_ = mx.symbol.Activation(data=batchnorm52_1_5_,
            act_type='relu',
            name="relu52_1_5_")

        conv53_1_5_ = mx.symbol.Convolution(data=relu52_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_5_")
        # conv53_1_5_, output shape: {[1024,14,14]}

        batchnorm53_1_5_ = mx.symbol.BatchNorm(data=conv53_1_5_,
            fix_gamma=True,
            name="batchnorm53_1_5_")
        conv51_1_6_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_6_")
        # conv51_1_6_, output shape: {[16,14,14]}

        batchnorm51_1_6_ = mx.symbol.BatchNorm(data=conv51_1_6_,
            fix_gamma=True,
            name="batchnorm51_1_6_")
        relu51_1_6_ = mx.symbol.Activation(data=batchnorm51_1_6_,
            act_type='relu',
            name="relu51_1_6_")

        conv52_1_6_ = mx.symbol.pad(data=relu51_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_6_ = mx.symbol.Convolution(data=conv52_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_6_")
        # conv52_1_6_, output shape: {[16,14,14]}

        batchnorm52_1_6_ = mx.symbol.BatchNorm(data=conv52_1_6_,
            fix_gamma=True,
            name="batchnorm52_1_6_")
        relu52_1_6_ = mx.symbol.Activation(data=batchnorm52_1_6_,
            act_type='relu',
            name="relu52_1_6_")

        conv53_1_6_ = mx.symbol.Convolution(data=relu52_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_6_")
        # conv53_1_6_, output shape: {[1024,14,14]}

        batchnorm53_1_6_ = mx.symbol.BatchNorm(data=conv53_1_6_,
            fix_gamma=True,
            name="batchnorm53_1_6_")
        conv51_1_7_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_7_")
        # conv51_1_7_, output shape: {[16,14,14]}

        batchnorm51_1_7_ = mx.symbol.BatchNorm(data=conv51_1_7_,
            fix_gamma=True,
            name="batchnorm51_1_7_")
        relu51_1_7_ = mx.symbol.Activation(data=batchnorm51_1_7_,
            act_type='relu',
            name="relu51_1_7_")

        conv52_1_7_ = mx.symbol.pad(data=relu51_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_7_ = mx.symbol.Convolution(data=conv52_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_7_")
        # conv52_1_7_, output shape: {[16,14,14]}

        batchnorm52_1_7_ = mx.symbol.BatchNorm(data=conv52_1_7_,
            fix_gamma=True,
            name="batchnorm52_1_7_")
        relu52_1_7_ = mx.symbol.Activation(data=batchnorm52_1_7_,
            act_type='relu',
            name="relu52_1_7_")

        conv53_1_7_ = mx.symbol.Convolution(data=relu52_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_7_")
        # conv53_1_7_, output shape: {[1024,14,14]}

        batchnorm53_1_7_ = mx.symbol.BatchNorm(data=conv53_1_7_,
            fix_gamma=True,
            name="batchnorm53_1_7_")
        conv51_1_8_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_8_")
        # conv51_1_8_, output shape: {[16,14,14]}

        batchnorm51_1_8_ = mx.symbol.BatchNorm(data=conv51_1_8_,
            fix_gamma=True,
            name="batchnorm51_1_8_")
        relu51_1_8_ = mx.symbol.Activation(data=batchnorm51_1_8_,
            act_type='relu',
            name="relu51_1_8_")

        conv52_1_8_ = mx.symbol.pad(data=relu51_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_8_ = mx.symbol.Convolution(data=conv52_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_8_")
        # conv52_1_8_, output shape: {[16,14,14]}

        batchnorm52_1_8_ = mx.symbol.BatchNorm(data=conv52_1_8_,
            fix_gamma=True,
            name="batchnorm52_1_8_")
        relu52_1_8_ = mx.symbol.Activation(data=batchnorm52_1_8_,
            act_type='relu',
            name="relu52_1_8_")

        conv53_1_8_ = mx.symbol.Convolution(data=relu52_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_8_")
        # conv53_1_8_, output shape: {[1024,14,14]}

        batchnorm53_1_8_ = mx.symbol.BatchNorm(data=conv53_1_8_,
            fix_gamma=True,
            name="batchnorm53_1_8_")
        conv51_1_9_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_9_")
        # conv51_1_9_, output shape: {[16,14,14]}

        batchnorm51_1_9_ = mx.symbol.BatchNorm(data=conv51_1_9_,
            fix_gamma=True,
            name="batchnorm51_1_9_")
        relu51_1_9_ = mx.symbol.Activation(data=batchnorm51_1_9_,
            act_type='relu',
            name="relu51_1_9_")

        conv52_1_9_ = mx.symbol.pad(data=relu51_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_9_ = mx.symbol.Convolution(data=conv52_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_9_")
        # conv52_1_9_, output shape: {[16,14,14]}

        batchnorm52_1_9_ = mx.symbol.BatchNorm(data=conv52_1_9_,
            fix_gamma=True,
            name="batchnorm52_1_9_")
        relu52_1_9_ = mx.symbol.Activation(data=batchnorm52_1_9_,
            act_type='relu',
            name="relu52_1_9_")

        conv53_1_9_ = mx.symbol.Convolution(data=relu52_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_9_")
        # conv53_1_9_, output shape: {[1024,14,14]}

        batchnorm53_1_9_ = mx.symbol.BatchNorm(data=conv53_1_9_,
            fix_gamma=True,
            name="batchnorm53_1_9_")
        conv51_1_10_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_10_")
        # conv51_1_10_, output shape: {[16,14,14]}

        batchnorm51_1_10_ = mx.symbol.BatchNorm(data=conv51_1_10_,
            fix_gamma=True,
            name="batchnorm51_1_10_")
        relu51_1_10_ = mx.symbol.Activation(data=batchnorm51_1_10_,
            act_type='relu',
            name="relu51_1_10_")

        conv52_1_10_ = mx.symbol.pad(data=relu51_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_10_ = mx.symbol.Convolution(data=conv52_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_10_")
        # conv52_1_10_, output shape: {[16,14,14]}

        batchnorm52_1_10_ = mx.symbol.BatchNorm(data=conv52_1_10_,
            fix_gamma=True,
            name="batchnorm52_1_10_")
        relu52_1_10_ = mx.symbol.Activation(data=batchnorm52_1_10_,
            act_type='relu',
            name="relu52_1_10_")

        conv53_1_10_ = mx.symbol.Convolution(data=relu52_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_10_")
        # conv53_1_10_, output shape: {[1024,14,14]}

        batchnorm53_1_10_ = mx.symbol.BatchNorm(data=conv53_1_10_,
            fix_gamma=True,
            name="batchnorm53_1_10_")
        conv51_1_11_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_11_")
        # conv51_1_11_, output shape: {[16,14,14]}

        batchnorm51_1_11_ = mx.symbol.BatchNorm(data=conv51_1_11_,
            fix_gamma=True,
            name="batchnorm51_1_11_")
        relu51_1_11_ = mx.symbol.Activation(data=batchnorm51_1_11_,
            act_type='relu',
            name="relu51_1_11_")

        conv52_1_11_ = mx.symbol.pad(data=relu51_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_11_ = mx.symbol.Convolution(data=conv52_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_11_")
        # conv52_1_11_, output shape: {[16,14,14]}

        batchnorm52_1_11_ = mx.symbol.BatchNorm(data=conv52_1_11_,
            fix_gamma=True,
            name="batchnorm52_1_11_")
        relu52_1_11_ = mx.symbol.Activation(data=batchnorm52_1_11_,
            act_type='relu',
            name="relu52_1_11_")

        conv53_1_11_ = mx.symbol.Convolution(data=relu52_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_11_")
        # conv53_1_11_, output shape: {[1024,14,14]}

        batchnorm53_1_11_ = mx.symbol.BatchNorm(data=conv53_1_11_,
            fix_gamma=True,
            name="batchnorm53_1_11_")
        conv51_1_12_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_12_")
        # conv51_1_12_, output shape: {[16,14,14]}

        batchnorm51_1_12_ = mx.symbol.BatchNorm(data=conv51_1_12_,
            fix_gamma=True,
            name="batchnorm51_1_12_")
        relu51_1_12_ = mx.symbol.Activation(data=batchnorm51_1_12_,
            act_type='relu',
            name="relu51_1_12_")

        conv52_1_12_ = mx.symbol.pad(data=relu51_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_12_ = mx.symbol.Convolution(data=conv52_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_12_")
        # conv52_1_12_, output shape: {[16,14,14]}

        batchnorm52_1_12_ = mx.symbol.BatchNorm(data=conv52_1_12_,
            fix_gamma=True,
            name="batchnorm52_1_12_")
        relu52_1_12_ = mx.symbol.Activation(data=batchnorm52_1_12_,
            act_type='relu',
            name="relu52_1_12_")

        conv53_1_12_ = mx.symbol.Convolution(data=relu52_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_12_")
        # conv53_1_12_, output shape: {[1024,14,14]}

        batchnorm53_1_12_ = mx.symbol.BatchNorm(data=conv53_1_12_,
            fix_gamma=True,
            name="batchnorm53_1_12_")
        conv51_1_13_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_13_")
        # conv51_1_13_, output shape: {[16,14,14]}

        batchnorm51_1_13_ = mx.symbol.BatchNorm(data=conv51_1_13_,
            fix_gamma=True,
            name="batchnorm51_1_13_")
        relu51_1_13_ = mx.symbol.Activation(data=batchnorm51_1_13_,
            act_type='relu',
            name="relu51_1_13_")

        conv52_1_13_ = mx.symbol.pad(data=relu51_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_13_ = mx.symbol.Convolution(data=conv52_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_13_")
        # conv52_1_13_, output shape: {[16,14,14]}

        batchnorm52_1_13_ = mx.symbol.BatchNorm(data=conv52_1_13_,
            fix_gamma=True,
            name="batchnorm52_1_13_")
        relu52_1_13_ = mx.symbol.Activation(data=batchnorm52_1_13_,
            act_type='relu',
            name="relu52_1_13_")

        conv53_1_13_ = mx.symbol.Convolution(data=relu52_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_13_")
        # conv53_1_13_, output shape: {[1024,14,14]}

        batchnorm53_1_13_ = mx.symbol.BatchNorm(data=conv53_1_13_,
            fix_gamma=True,
            name="batchnorm53_1_13_")
        conv51_1_14_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_14_")
        # conv51_1_14_, output shape: {[16,14,14]}

        batchnorm51_1_14_ = mx.symbol.BatchNorm(data=conv51_1_14_,
            fix_gamma=True,
            name="batchnorm51_1_14_")
        relu51_1_14_ = mx.symbol.Activation(data=batchnorm51_1_14_,
            act_type='relu',
            name="relu51_1_14_")

        conv52_1_14_ = mx.symbol.pad(data=relu51_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_14_ = mx.symbol.Convolution(data=conv52_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_14_")
        # conv52_1_14_, output shape: {[16,14,14]}

        batchnorm52_1_14_ = mx.symbol.BatchNorm(data=conv52_1_14_,
            fix_gamma=True,
            name="batchnorm52_1_14_")
        relu52_1_14_ = mx.symbol.Activation(data=batchnorm52_1_14_,
            act_type='relu',
            name="relu52_1_14_")

        conv53_1_14_ = mx.symbol.Convolution(data=relu52_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_14_")
        # conv53_1_14_, output shape: {[1024,14,14]}

        batchnorm53_1_14_ = mx.symbol.BatchNorm(data=conv53_1_14_,
            fix_gamma=True,
            name="batchnorm53_1_14_")
        conv51_1_15_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_15_")
        # conv51_1_15_, output shape: {[16,14,14]}

        batchnorm51_1_15_ = mx.symbol.BatchNorm(data=conv51_1_15_,
            fix_gamma=True,
            name="batchnorm51_1_15_")
        relu51_1_15_ = mx.symbol.Activation(data=batchnorm51_1_15_,
            act_type='relu',
            name="relu51_1_15_")

        conv52_1_15_ = mx.symbol.pad(data=relu51_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_15_ = mx.symbol.Convolution(data=conv52_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_15_")
        # conv52_1_15_, output shape: {[16,14,14]}

        batchnorm52_1_15_ = mx.symbol.BatchNorm(data=conv52_1_15_,
            fix_gamma=True,
            name="batchnorm52_1_15_")
        relu52_1_15_ = mx.symbol.Activation(data=batchnorm52_1_15_,
            act_type='relu',
            name="relu52_1_15_")

        conv53_1_15_ = mx.symbol.Convolution(data=relu52_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_15_")
        # conv53_1_15_, output shape: {[1024,14,14]}

        batchnorm53_1_15_ = mx.symbol.BatchNorm(data=conv53_1_15_,
            fix_gamma=True,
            name="batchnorm53_1_15_")
        conv51_1_16_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_16_")
        # conv51_1_16_, output shape: {[16,14,14]}

        batchnorm51_1_16_ = mx.symbol.BatchNorm(data=conv51_1_16_,
            fix_gamma=True,
            name="batchnorm51_1_16_")
        relu51_1_16_ = mx.symbol.Activation(data=batchnorm51_1_16_,
            act_type='relu',
            name="relu51_1_16_")

        conv52_1_16_ = mx.symbol.pad(data=relu51_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_16_ = mx.symbol.Convolution(data=conv52_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_16_")
        # conv52_1_16_, output shape: {[16,14,14]}

        batchnorm52_1_16_ = mx.symbol.BatchNorm(data=conv52_1_16_,
            fix_gamma=True,
            name="batchnorm52_1_16_")
        relu52_1_16_ = mx.symbol.Activation(data=batchnorm52_1_16_,
            act_type='relu',
            name="relu52_1_16_")

        conv53_1_16_ = mx.symbol.Convolution(data=relu52_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_16_")
        # conv53_1_16_, output shape: {[1024,14,14]}

        batchnorm53_1_16_ = mx.symbol.BatchNorm(data=conv53_1_16_,
            fix_gamma=True,
            name="batchnorm53_1_16_")
        conv51_1_17_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_17_")
        # conv51_1_17_, output shape: {[16,14,14]}

        batchnorm51_1_17_ = mx.symbol.BatchNorm(data=conv51_1_17_,
            fix_gamma=True,
            name="batchnorm51_1_17_")
        relu51_1_17_ = mx.symbol.Activation(data=batchnorm51_1_17_,
            act_type='relu',
            name="relu51_1_17_")

        conv52_1_17_ = mx.symbol.pad(data=relu51_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_17_ = mx.symbol.Convolution(data=conv52_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_17_")
        # conv52_1_17_, output shape: {[16,14,14]}

        batchnorm52_1_17_ = mx.symbol.BatchNorm(data=conv52_1_17_,
            fix_gamma=True,
            name="batchnorm52_1_17_")
        relu52_1_17_ = mx.symbol.Activation(data=batchnorm52_1_17_,
            act_type='relu',
            name="relu52_1_17_")

        conv53_1_17_ = mx.symbol.Convolution(data=relu52_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_17_")
        # conv53_1_17_, output shape: {[1024,14,14]}

        batchnorm53_1_17_ = mx.symbol.BatchNorm(data=conv53_1_17_,
            fix_gamma=True,
            name="batchnorm53_1_17_")
        conv51_1_18_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_18_")
        # conv51_1_18_, output shape: {[16,14,14]}

        batchnorm51_1_18_ = mx.symbol.BatchNorm(data=conv51_1_18_,
            fix_gamma=True,
            name="batchnorm51_1_18_")
        relu51_1_18_ = mx.symbol.Activation(data=batchnorm51_1_18_,
            act_type='relu',
            name="relu51_1_18_")

        conv52_1_18_ = mx.symbol.pad(data=relu51_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_18_ = mx.symbol.Convolution(data=conv52_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_18_")
        # conv52_1_18_, output shape: {[16,14,14]}

        batchnorm52_1_18_ = mx.symbol.BatchNorm(data=conv52_1_18_,
            fix_gamma=True,
            name="batchnorm52_1_18_")
        relu52_1_18_ = mx.symbol.Activation(data=batchnorm52_1_18_,
            act_type='relu',
            name="relu52_1_18_")

        conv53_1_18_ = mx.symbol.Convolution(data=relu52_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_18_")
        # conv53_1_18_, output shape: {[1024,14,14]}

        batchnorm53_1_18_ = mx.symbol.BatchNorm(data=conv53_1_18_,
            fix_gamma=True,
            name="batchnorm53_1_18_")
        conv51_1_19_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_19_")
        # conv51_1_19_, output shape: {[16,14,14]}

        batchnorm51_1_19_ = mx.symbol.BatchNorm(data=conv51_1_19_,
            fix_gamma=True,
            name="batchnorm51_1_19_")
        relu51_1_19_ = mx.symbol.Activation(data=batchnorm51_1_19_,
            act_type='relu',
            name="relu51_1_19_")

        conv52_1_19_ = mx.symbol.pad(data=relu51_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_19_ = mx.symbol.Convolution(data=conv52_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_19_")
        # conv52_1_19_, output shape: {[16,14,14]}

        batchnorm52_1_19_ = mx.symbol.BatchNorm(data=conv52_1_19_,
            fix_gamma=True,
            name="batchnorm52_1_19_")
        relu52_1_19_ = mx.symbol.Activation(data=batchnorm52_1_19_,
            act_type='relu',
            name="relu52_1_19_")

        conv53_1_19_ = mx.symbol.Convolution(data=relu52_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_19_")
        # conv53_1_19_, output shape: {[1024,14,14]}

        batchnorm53_1_19_ = mx.symbol.BatchNorm(data=conv53_1_19_,
            fix_gamma=True,
            name="batchnorm53_1_19_")
        conv51_1_20_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_20_")
        # conv51_1_20_, output shape: {[16,14,14]}

        batchnorm51_1_20_ = mx.symbol.BatchNorm(data=conv51_1_20_,
            fix_gamma=True,
            name="batchnorm51_1_20_")
        relu51_1_20_ = mx.symbol.Activation(data=batchnorm51_1_20_,
            act_type='relu',
            name="relu51_1_20_")

        conv52_1_20_ = mx.symbol.pad(data=relu51_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_20_ = mx.symbol.Convolution(data=conv52_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_20_")
        # conv52_1_20_, output shape: {[16,14,14]}

        batchnorm52_1_20_ = mx.symbol.BatchNorm(data=conv52_1_20_,
            fix_gamma=True,
            name="batchnorm52_1_20_")
        relu52_1_20_ = mx.symbol.Activation(data=batchnorm52_1_20_,
            act_type='relu',
            name="relu52_1_20_")

        conv53_1_20_ = mx.symbol.Convolution(data=relu52_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_20_")
        # conv53_1_20_, output shape: {[1024,14,14]}

        batchnorm53_1_20_ = mx.symbol.BatchNorm(data=conv53_1_20_,
            fix_gamma=True,
            name="batchnorm53_1_20_")
        conv51_1_21_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_21_")
        # conv51_1_21_, output shape: {[16,14,14]}

        batchnorm51_1_21_ = mx.symbol.BatchNorm(data=conv51_1_21_,
            fix_gamma=True,
            name="batchnorm51_1_21_")
        relu51_1_21_ = mx.symbol.Activation(data=batchnorm51_1_21_,
            act_type='relu',
            name="relu51_1_21_")

        conv52_1_21_ = mx.symbol.pad(data=relu51_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_21_ = mx.symbol.Convolution(data=conv52_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_21_")
        # conv52_1_21_, output shape: {[16,14,14]}

        batchnorm52_1_21_ = mx.symbol.BatchNorm(data=conv52_1_21_,
            fix_gamma=True,
            name="batchnorm52_1_21_")
        relu52_1_21_ = mx.symbol.Activation(data=batchnorm52_1_21_,
            act_type='relu',
            name="relu52_1_21_")

        conv53_1_21_ = mx.symbol.Convolution(data=relu52_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_21_")
        # conv53_1_21_, output shape: {[1024,14,14]}

        batchnorm53_1_21_ = mx.symbol.BatchNorm(data=conv53_1_21_,
            fix_gamma=True,
            name="batchnorm53_1_21_")
        conv51_1_22_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_22_")
        # conv51_1_22_, output shape: {[16,14,14]}

        batchnorm51_1_22_ = mx.symbol.BatchNorm(data=conv51_1_22_,
            fix_gamma=True,
            name="batchnorm51_1_22_")
        relu51_1_22_ = mx.symbol.Activation(data=batchnorm51_1_22_,
            act_type='relu',
            name="relu51_1_22_")

        conv52_1_22_ = mx.symbol.pad(data=relu51_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_22_ = mx.symbol.Convolution(data=conv52_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_22_")
        # conv52_1_22_, output shape: {[16,14,14]}

        batchnorm52_1_22_ = mx.symbol.BatchNorm(data=conv52_1_22_,
            fix_gamma=True,
            name="batchnorm52_1_22_")
        relu52_1_22_ = mx.symbol.Activation(data=batchnorm52_1_22_,
            act_type='relu',
            name="relu52_1_22_")

        conv53_1_22_ = mx.symbol.Convolution(data=relu52_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_22_")
        # conv53_1_22_, output shape: {[1024,14,14]}

        batchnorm53_1_22_ = mx.symbol.BatchNorm(data=conv53_1_22_,
            fix_gamma=True,
            name="batchnorm53_1_22_")
        conv51_1_23_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_23_")
        # conv51_1_23_, output shape: {[16,14,14]}

        batchnorm51_1_23_ = mx.symbol.BatchNorm(data=conv51_1_23_,
            fix_gamma=True,
            name="batchnorm51_1_23_")
        relu51_1_23_ = mx.symbol.Activation(data=batchnorm51_1_23_,
            act_type='relu',
            name="relu51_1_23_")

        conv52_1_23_ = mx.symbol.pad(data=relu51_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_23_ = mx.symbol.Convolution(data=conv52_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_23_")
        # conv52_1_23_, output shape: {[16,14,14]}

        batchnorm52_1_23_ = mx.symbol.BatchNorm(data=conv52_1_23_,
            fix_gamma=True,
            name="batchnorm52_1_23_")
        relu52_1_23_ = mx.symbol.Activation(data=batchnorm52_1_23_,
            act_type='relu',
            name="relu52_1_23_")

        conv53_1_23_ = mx.symbol.Convolution(data=relu52_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_23_")
        # conv53_1_23_, output shape: {[1024,14,14]}

        batchnorm53_1_23_ = mx.symbol.BatchNorm(data=conv53_1_23_,
            fix_gamma=True,
            name="batchnorm53_1_23_")
        conv51_1_24_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_24_")
        # conv51_1_24_, output shape: {[16,14,14]}

        batchnorm51_1_24_ = mx.symbol.BatchNorm(data=conv51_1_24_,
            fix_gamma=True,
            name="batchnorm51_1_24_")
        relu51_1_24_ = mx.symbol.Activation(data=batchnorm51_1_24_,
            act_type='relu',
            name="relu51_1_24_")

        conv52_1_24_ = mx.symbol.pad(data=relu51_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_24_ = mx.symbol.Convolution(data=conv52_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_24_")
        # conv52_1_24_, output shape: {[16,14,14]}

        batchnorm52_1_24_ = mx.symbol.BatchNorm(data=conv52_1_24_,
            fix_gamma=True,
            name="batchnorm52_1_24_")
        relu52_1_24_ = mx.symbol.Activation(data=batchnorm52_1_24_,
            act_type='relu',
            name="relu52_1_24_")

        conv53_1_24_ = mx.symbol.Convolution(data=relu52_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_24_")
        # conv53_1_24_, output shape: {[1024,14,14]}

        batchnorm53_1_24_ = mx.symbol.BatchNorm(data=conv53_1_24_,
            fix_gamma=True,
            name="batchnorm53_1_24_")
        conv51_1_25_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_25_")
        # conv51_1_25_, output shape: {[16,14,14]}

        batchnorm51_1_25_ = mx.symbol.BatchNorm(data=conv51_1_25_,
            fix_gamma=True,
            name="batchnorm51_1_25_")
        relu51_1_25_ = mx.symbol.Activation(data=batchnorm51_1_25_,
            act_type='relu',
            name="relu51_1_25_")

        conv52_1_25_ = mx.symbol.pad(data=relu51_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_25_ = mx.symbol.Convolution(data=conv52_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_25_")
        # conv52_1_25_, output shape: {[16,14,14]}

        batchnorm52_1_25_ = mx.symbol.BatchNorm(data=conv52_1_25_,
            fix_gamma=True,
            name="batchnorm52_1_25_")
        relu52_1_25_ = mx.symbol.Activation(data=batchnorm52_1_25_,
            act_type='relu',
            name="relu52_1_25_")

        conv53_1_25_ = mx.symbol.Convolution(data=relu52_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_25_")
        # conv53_1_25_, output shape: {[1024,14,14]}

        batchnorm53_1_25_ = mx.symbol.BatchNorm(data=conv53_1_25_,
            fix_gamma=True,
            name="batchnorm53_1_25_")
        conv51_1_26_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_26_")
        # conv51_1_26_, output shape: {[16,14,14]}

        batchnorm51_1_26_ = mx.symbol.BatchNorm(data=conv51_1_26_,
            fix_gamma=True,
            name="batchnorm51_1_26_")
        relu51_1_26_ = mx.symbol.Activation(data=batchnorm51_1_26_,
            act_type='relu',
            name="relu51_1_26_")

        conv52_1_26_ = mx.symbol.pad(data=relu51_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_26_ = mx.symbol.Convolution(data=conv52_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_26_")
        # conv52_1_26_, output shape: {[16,14,14]}

        batchnorm52_1_26_ = mx.symbol.BatchNorm(data=conv52_1_26_,
            fix_gamma=True,
            name="batchnorm52_1_26_")
        relu52_1_26_ = mx.symbol.Activation(data=batchnorm52_1_26_,
            act_type='relu',
            name="relu52_1_26_")

        conv53_1_26_ = mx.symbol.Convolution(data=relu52_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_26_")
        # conv53_1_26_, output shape: {[1024,14,14]}

        batchnorm53_1_26_ = mx.symbol.BatchNorm(data=conv53_1_26_,
            fix_gamma=True,
            name="batchnorm53_1_26_")
        conv51_1_27_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_27_")
        # conv51_1_27_, output shape: {[16,14,14]}

        batchnorm51_1_27_ = mx.symbol.BatchNorm(data=conv51_1_27_,
            fix_gamma=True,
            name="batchnorm51_1_27_")
        relu51_1_27_ = mx.symbol.Activation(data=batchnorm51_1_27_,
            act_type='relu',
            name="relu51_1_27_")

        conv52_1_27_ = mx.symbol.pad(data=relu51_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_27_ = mx.symbol.Convolution(data=conv52_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_27_")
        # conv52_1_27_, output shape: {[16,14,14]}

        batchnorm52_1_27_ = mx.symbol.BatchNorm(data=conv52_1_27_,
            fix_gamma=True,
            name="batchnorm52_1_27_")
        relu52_1_27_ = mx.symbol.Activation(data=batchnorm52_1_27_,
            act_type='relu',
            name="relu52_1_27_")

        conv53_1_27_ = mx.symbol.Convolution(data=relu52_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_27_")
        # conv53_1_27_, output shape: {[1024,14,14]}

        batchnorm53_1_27_ = mx.symbol.BatchNorm(data=conv53_1_27_,
            fix_gamma=True,
            name="batchnorm53_1_27_")
        conv51_1_28_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_28_")
        # conv51_1_28_, output shape: {[16,14,14]}

        batchnorm51_1_28_ = mx.symbol.BatchNorm(data=conv51_1_28_,
            fix_gamma=True,
            name="batchnorm51_1_28_")
        relu51_1_28_ = mx.symbol.Activation(data=batchnorm51_1_28_,
            act_type='relu',
            name="relu51_1_28_")

        conv52_1_28_ = mx.symbol.pad(data=relu51_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_28_ = mx.symbol.Convolution(data=conv52_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_28_")
        # conv52_1_28_, output shape: {[16,14,14]}

        batchnorm52_1_28_ = mx.symbol.BatchNorm(data=conv52_1_28_,
            fix_gamma=True,
            name="batchnorm52_1_28_")
        relu52_1_28_ = mx.symbol.Activation(data=batchnorm52_1_28_,
            act_type='relu',
            name="relu52_1_28_")

        conv53_1_28_ = mx.symbol.Convolution(data=relu52_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_28_")
        # conv53_1_28_, output shape: {[1024,14,14]}

        batchnorm53_1_28_ = mx.symbol.BatchNorm(data=conv53_1_28_,
            fix_gamma=True,
            name="batchnorm53_1_28_")
        conv51_1_29_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_29_")
        # conv51_1_29_, output shape: {[16,14,14]}

        batchnorm51_1_29_ = mx.symbol.BatchNorm(data=conv51_1_29_,
            fix_gamma=True,
            name="batchnorm51_1_29_")
        relu51_1_29_ = mx.symbol.Activation(data=batchnorm51_1_29_,
            act_type='relu',
            name="relu51_1_29_")

        conv52_1_29_ = mx.symbol.pad(data=relu51_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_29_ = mx.symbol.Convolution(data=conv52_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_29_")
        # conv52_1_29_, output shape: {[16,14,14]}

        batchnorm52_1_29_ = mx.symbol.BatchNorm(data=conv52_1_29_,
            fix_gamma=True,
            name="batchnorm52_1_29_")
        relu52_1_29_ = mx.symbol.Activation(data=batchnorm52_1_29_,
            act_type='relu',
            name="relu52_1_29_")

        conv53_1_29_ = mx.symbol.Convolution(data=relu52_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_29_")
        # conv53_1_29_, output shape: {[1024,14,14]}

        batchnorm53_1_29_ = mx.symbol.BatchNorm(data=conv53_1_29_,
            fix_gamma=True,
            name="batchnorm53_1_29_")
        conv51_1_30_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_30_")
        # conv51_1_30_, output shape: {[16,14,14]}

        batchnorm51_1_30_ = mx.symbol.BatchNorm(data=conv51_1_30_,
            fix_gamma=True,
            name="batchnorm51_1_30_")
        relu51_1_30_ = mx.symbol.Activation(data=batchnorm51_1_30_,
            act_type='relu',
            name="relu51_1_30_")

        conv52_1_30_ = mx.symbol.pad(data=relu51_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_30_ = mx.symbol.Convolution(data=conv52_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_30_")
        # conv52_1_30_, output shape: {[16,14,14]}

        batchnorm52_1_30_ = mx.symbol.BatchNorm(data=conv52_1_30_,
            fix_gamma=True,
            name="batchnorm52_1_30_")
        relu52_1_30_ = mx.symbol.Activation(data=batchnorm52_1_30_,
            act_type='relu',
            name="relu52_1_30_")

        conv53_1_30_ = mx.symbol.Convolution(data=relu52_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_30_")
        # conv53_1_30_, output shape: {[1024,14,14]}

        batchnorm53_1_30_ = mx.symbol.BatchNorm(data=conv53_1_30_,
            fix_gamma=True,
            name="batchnorm53_1_30_")
        conv51_1_31_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_31_")
        # conv51_1_31_, output shape: {[16,14,14]}

        batchnorm51_1_31_ = mx.symbol.BatchNorm(data=conv51_1_31_,
            fix_gamma=True,
            name="batchnorm51_1_31_")
        relu51_1_31_ = mx.symbol.Activation(data=batchnorm51_1_31_,
            act_type='relu',
            name="relu51_1_31_")

        conv52_1_31_ = mx.symbol.pad(data=relu51_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_31_ = mx.symbol.Convolution(data=conv52_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_31_")
        # conv52_1_31_, output shape: {[16,14,14]}

        batchnorm52_1_31_ = mx.symbol.BatchNorm(data=conv52_1_31_,
            fix_gamma=True,
            name="batchnorm52_1_31_")
        relu52_1_31_ = mx.symbol.Activation(data=batchnorm52_1_31_,
            act_type='relu',
            name="relu52_1_31_")

        conv53_1_31_ = mx.symbol.Convolution(data=relu52_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_31_")
        # conv53_1_31_, output shape: {[1024,14,14]}

        batchnorm53_1_31_ = mx.symbol.BatchNorm(data=conv53_1_31_,
            fix_gamma=True,
            name="batchnorm53_1_31_")
        conv51_1_32_ = mx.symbol.Convolution(data=relu49_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv51_1_32_")
        # conv51_1_32_, output shape: {[16,14,14]}

        batchnorm51_1_32_ = mx.symbol.BatchNorm(data=conv51_1_32_,
            fix_gamma=True,
            name="batchnorm51_1_32_")
        relu51_1_32_ = mx.symbol.Activation(data=batchnorm51_1_32_,
            act_type='relu',
            name="relu51_1_32_")

        conv52_1_32_ = mx.symbol.pad(data=relu51_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv52_1_32_ = mx.symbol.Convolution(data=conv52_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv52_1_32_")
        # conv52_1_32_, output shape: {[16,14,14]}

        batchnorm52_1_32_ = mx.symbol.BatchNorm(data=conv52_1_32_,
            fix_gamma=True,
            name="batchnorm52_1_32_")
        relu52_1_32_ = mx.symbol.Activation(data=batchnorm52_1_32_,
            act_type='relu',
            name="relu52_1_32_")

        conv53_1_32_ = mx.symbol.Convolution(data=relu52_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv53_1_32_")
        # conv53_1_32_, output shape: {[1024,14,14]}

        batchnorm53_1_32_ = mx.symbol.BatchNorm(data=conv53_1_32_,
            fix_gamma=True,
            name="batchnorm53_1_32_")
        add54_1_ = batchnorm53_1_1_ + batchnorm53_1_2_ + batchnorm53_1_3_ + batchnorm53_1_4_ + batchnorm53_1_5_ + batchnorm53_1_6_ + batchnorm53_1_7_ + batchnorm53_1_8_ + batchnorm53_1_9_ + batchnorm53_1_10_ + batchnorm53_1_11_ + batchnorm53_1_12_ + batchnorm53_1_13_ + batchnorm53_1_14_ + batchnorm53_1_15_ + batchnorm53_1_16_ + batchnorm53_1_17_ + batchnorm53_1_18_ + batchnorm53_1_19_ + batchnorm53_1_20_ + batchnorm53_1_21_ + batchnorm53_1_22_ + batchnorm53_1_23_ + batchnorm53_1_24_ + batchnorm53_1_25_ + batchnorm53_1_26_ + batchnorm53_1_27_ + batchnorm53_1_28_ + batchnorm53_1_29_ + batchnorm53_1_30_ + batchnorm53_1_31_ + batchnorm53_1_32_
        # add54_1_, output shape: {[1024,14,14]}

        add55_ = add54_1_ + relu49_
        # add55_, output shape: {[1024,14,14]}

        relu55_ = mx.symbol.Activation(data=add55_,
            act_type='relu',
            name="relu55_")

        conv57_1_1_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_1_")
        # conv57_1_1_, output shape: {[16,14,14]}

        batchnorm57_1_1_ = mx.symbol.BatchNorm(data=conv57_1_1_,
            fix_gamma=True,
            name="batchnorm57_1_1_")
        relu57_1_1_ = mx.symbol.Activation(data=batchnorm57_1_1_,
            act_type='relu',
            name="relu57_1_1_")

        conv58_1_1_ = mx.symbol.pad(data=relu57_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_1_ = mx.symbol.Convolution(data=conv58_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_1_")
        # conv58_1_1_, output shape: {[16,14,14]}

        batchnorm58_1_1_ = mx.symbol.BatchNorm(data=conv58_1_1_,
            fix_gamma=True,
            name="batchnorm58_1_1_")
        relu58_1_1_ = mx.symbol.Activation(data=batchnorm58_1_1_,
            act_type='relu',
            name="relu58_1_1_")

        conv59_1_1_ = mx.symbol.Convolution(data=relu58_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_1_")
        # conv59_1_1_, output shape: {[1024,14,14]}

        batchnorm59_1_1_ = mx.symbol.BatchNorm(data=conv59_1_1_,
            fix_gamma=True,
            name="batchnorm59_1_1_")
        conv57_1_2_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_2_")
        # conv57_1_2_, output shape: {[16,14,14]}

        batchnorm57_1_2_ = mx.symbol.BatchNorm(data=conv57_1_2_,
            fix_gamma=True,
            name="batchnorm57_1_2_")
        relu57_1_2_ = mx.symbol.Activation(data=batchnorm57_1_2_,
            act_type='relu',
            name="relu57_1_2_")

        conv58_1_2_ = mx.symbol.pad(data=relu57_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_2_ = mx.symbol.Convolution(data=conv58_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_2_")
        # conv58_1_2_, output shape: {[16,14,14]}

        batchnorm58_1_2_ = mx.symbol.BatchNorm(data=conv58_1_2_,
            fix_gamma=True,
            name="batchnorm58_1_2_")
        relu58_1_2_ = mx.symbol.Activation(data=batchnorm58_1_2_,
            act_type='relu',
            name="relu58_1_2_")

        conv59_1_2_ = mx.symbol.Convolution(data=relu58_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_2_")
        # conv59_1_2_, output shape: {[1024,14,14]}

        batchnorm59_1_2_ = mx.symbol.BatchNorm(data=conv59_1_2_,
            fix_gamma=True,
            name="batchnorm59_1_2_")
        conv57_1_3_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_3_")
        # conv57_1_3_, output shape: {[16,14,14]}

        batchnorm57_1_3_ = mx.symbol.BatchNorm(data=conv57_1_3_,
            fix_gamma=True,
            name="batchnorm57_1_3_")
        relu57_1_3_ = mx.symbol.Activation(data=batchnorm57_1_3_,
            act_type='relu',
            name="relu57_1_3_")

        conv58_1_3_ = mx.symbol.pad(data=relu57_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_3_ = mx.symbol.Convolution(data=conv58_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_3_")
        # conv58_1_3_, output shape: {[16,14,14]}

        batchnorm58_1_3_ = mx.symbol.BatchNorm(data=conv58_1_3_,
            fix_gamma=True,
            name="batchnorm58_1_3_")
        relu58_1_3_ = mx.symbol.Activation(data=batchnorm58_1_3_,
            act_type='relu',
            name="relu58_1_3_")

        conv59_1_3_ = mx.symbol.Convolution(data=relu58_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_3_")
        # conv59_1_3_, output shape: {[1024,14,14]}

        batchnorm59_1_3_ = mx.symbol.BatchNorm(data=conv59_1_3_,
            fix_gamma=True,
            name="batchnorm59_1_3_")
        conv57_1_4_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_4_")
        # conv57_1_4_, output shape: {[16,14,14]}

        batchnorm57_1_4_ = mx.symbol.BatchNorm(data=conv57_1_4_,
            fix_gamma=True,
            name="batchnorm57_1_4_")
        relu57_1_4_ = mx.symbol.Activation(data=batchnorm57_1_4_,
            act_type='relu',
            name="relu57_1_4_")

        conv58_1_4_ = mx.symbol.pad(data=relu57_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_4_ = mx.symbol.Convolution(data=conv58_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_4_")
        # conv58_1_4_, output shape: {[16,14,14]}

        batchnorm58_1_4_ = mx.symbol.BatchNorm(data=conv58_1_4_,
            fix_gamma=True,
            name="batchnorm58_1_4_")
        relu58_1_4_ = mx.symbol.Activation(data=batchnorm58_1_4_,
            act_type='relu',
            name="relu58_1_4_")

        conv59_1_4_ = mx.symbol.Convolution(data=relu58_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_4_")
        # conv59_1_4_, output shape: {[1024,14,14]}

        batchnorm59_1_4_ = mx.symbol.BatchNorm(data=conv59_1_4_,
            fix_gamma=True,
            name="batchnorm59_1_4_")
        conv57_1_5_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_5_")
        # conv57_1_5_, output shape: {[16,14,14]}

        batchnorm57_1_5_ = mx.symbol.BatchNorm(data=conv57_1_5_,
            fix_gamma=True,
            name="batchnorm57_1_5_")
        relu57_1_5_ = mx.symbol.Activation(data=batchnorm57_1_5_,
            act_type='relu',
            name="relu57_1_5_")

        conv58_1_5_ = mx.symbol.pad(data=relu57_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_5_ = mx.symbol.Convolution(data=conv58_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_5_")
        # conv58_1_5_, output shape: {[16,14,14]}

        batchnorm58_1_5_ = mx.symbol.BatchNorm(data=conv58_1_5_,
            fix_gamma=True,
            name="batchnorm58_1_5_")
        relu58_1_5_ = mx.symbol.Activation(data=batchnorm58_1_5_,
            act_type='relu',
            name="relu58_1_5_")

        conv59_1_5_ = mx.symbol.Convolution(data=relu58_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_5_")
        # conv59_1_5_, output shape: {[1024,14,14]}

        batchnorm59_1_5_ = mx.symbol.BatchNorm(data=conv59_1_5_,
            fix_gamma=True,
            name="batchnorm59_1_5_")
        conv57_1_6_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_6_")
        # conv57_1_6_, output shape: {[16,14,14]}

        batchnorm57_1_6_ = mx.symbol.BatchNorm(data=conv57_1_6_,
            fix_gamma=True,
            name="batchnorm57_1_6_")
        relu57_1_6_ = mx.symbol.Activation(data=batchnorm57_1_6_,
            act_type='relu',
            name="relu57_1_6_")

        conv58_1_6_ = mx.symbol.pad(data=relu57_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_6_ = mx.symbol.Convolution(data=conv58_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_6_")
        # conv58_1_6_, output shape: {[16,14,14]}

        batchnorm58_1_6_ = mx.symbol.BatchNorm(data=conv58_1_6_,
            fix_gamma=True,
            name="batchnorm58_1_6_")
        relu58_1_6_ = mx.symbol.Activation(data=batchnorm58_1_6_,
            act_type='relu',
            name="relu58_1_6_")

        conv59_1_6_ = mx.symbol.Convolution(data=relu58_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_6_")
        # conv59_1_6_, output shape: {[1024,14,14]}

        batchnorm59_1_6_ = mx.symbol.BatchNorm(data=conv59_1_6_,
            fix_gamma=True,
            name="batchnorm59_1_6_")
        conv57_1_7_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_7_")
        # conv57_1_7_, output shape: {[16,14,14]}

        batchnorm57_1_7_ = mx.symbol.BatchNorm(data=conv57_1_7_,
            fix_gamma=True,
            name="batchnorm57_1_7_")
        relu57_1_7_ = mx.symbol.Activation(data=batchnorm57_1_7_,
            act_type='relu',
            name="relu57_1_7_")

        conv58_1_7_ = mx.symbol.pad(data=relu57_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_7_ = mx.symbol.Convolution(data=conv58_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_7_")
        # conv58_1_7_, output shape: {[16,14,14]}

        batchnorm58_1_7_ = mx.symbol.BatchNorm(data=conv58_1_7_,
            fix_gamma=True,
            name="batchnorm58_1_7_")
        relu58_1_7_ = mx.symbol.Activation(data=batchnorm58_1_7_,
            act_type='relu',
            name="relu58_1_7_")

        conv59_1_7_ = mx.symbol.Convolution(data=relu58_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_7_")
        # conv59_1_7_, output shape: {[1024,14,14]}

        batchnorm59_1_7_ = mx.symbol.BatchNorm(data=conv59_1_7_,
            fix_gamma=True,
            name="batchnorm59_1_7_")
        conv57_1_8_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_8_")
        # conv57_1_8_, output shape: {[16,14,14]}

        batchnorm57_1_8_ = mx.symbol.BatchNorm(data=conv57_1_8_,
            fix_gamma=True,
            name="batchnorm57_1_8_")
        relu57_1_8_ = mx.symbol.Activation(data=batchnorm57_1_8_,
            act_type='relu',
            name="relu57_1_8_")

        conv58_1_8_ = mx.symbol.pad(data=relu57_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_8_ = mx.symbol.Convolution(data=conv58_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_8_")
        # conv58_1_8_, output shape: {[16,14,14]}

        batchnorm58_1_8_ = mx.symbol.BatchNorm(data=conv58_1_8_,
            fix_gamma=True,
            name="batchnorm58_1_8_")
        relu58_1_8_ = mx.symbol.Activation(data=batchnorm58_1_8_,
            act_type='relu',
            name="relu58_1_8_")

        conv59_1_8_ = mx.symbol.Convolution(data=relu58_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_8_")
        # conv59_1_8_, output shape: {[1024,14,14]}

        batchnorm59_1_8_ = mx.symbol.BatchNorm(data=conv59_1_8_,
            fix_gamma=True,
            name="batchnorm59_1_8_")
        conv57_1_9_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_9_")
        # conv57_1_9_, output shape: {[16,14,14]}

        batchnorm57_1_9_ = mx.symbol.BatchNorm(data=conv57_1_9_,
            fix_gamma=True,
            name="batchnorm57_1_9_")
        relu57_1_9_ = mx.symbol.Activation(data=batchnorm57_1_9_,
            act_type='relu',
            name="relu57_1_9_")

        conv58_1_9_ = mx.symbol.pad(data=relu57_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_9_ = mx.symbol.Convolution(data=conv58_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_9_")
        # conv58_1_9_, output shape: {[16,14,14]}

        batchnorm58_1_9_ = mx.symbol.BatchNorm(data=conv58_1_9_,
            fix_gamma=True,
            name="batchnorm58_1_9_")
        relu58_1_9_ = mx.symbol.Activation(data=batchnorm58_1_9_,
            act_type='relu',
            name="relu58_1_9_")

        conv59_1_9_ = mx.symbol.Convolution(data=relu58_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_9_")
        # conv59_1_9_, output shape: {[1024,14,14]}

        batchnorm59_1_9_ = mx.symbol.BatchNorm(data=conv59_1_9_,
            fix_gamma=True,
            name="batchnorm59_1_9_")
        conv57_1_10_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_10_")
        # conv57_1_10_, output shape: {[16,14,14]}

        batchnorm57_1_10_ = mx.symbol.BatchNorm(data=conv57_1_10_,
            fix_gamma=True,
            name="batchnorm57_1_10_")
        relu57_1_10_ = mx.symbol.Activation(data=batchnorm57_1_10_,
            act_type='relu',
            name="relu57_1_10_")

        conv58_1_10_ = mx.symbol.pad(data=relu57_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_10_ = mx.symbol.Convolution(data=conv58_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_10_")
        # conv58_1_10_, output shape: {[16,14,14]}

        batchnorm58_1_10_ = mx.symbol.BatchNorm(data=conv58_1_10_,
            fix_gamma=True,
            name="batchnorm58_1_10_")
        relu58_1_10_ = mx.symbol.Activation(data=batchnorm58_1_10_,
            act_type='relu',
            name="relu58_1_10_")

        conv59_1_10_ = mx.symbol.Convolution(data=relu58_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_10_")
        # conv59_1_10_, output shape: {[1024,14,14]}

        batchnorm59_1_10_ = mx.symbol.BatchNorm(data=conv59_1_10_,
            fix_gamma=True,
            name="batchnorm59_1_10_")
        conv57_1_11_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_11_")
        # conv57_1_11_, output shape: {[16,14,14]}

        batchnorm57_1_11_ = mx.symbol.BatchNorm(data=conv57_1_11_,
            fix_gamma=True,
            name="batchnorm57_1_11_")
        relu57_1_11_ = mx.symbol.Activation(data=batchnorm57_1_11_,
            act_type='relu',
            name="relu57_1_11_")

        conv58_1_11_ = mx.symbol.pad(data=relu57_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_11_ = mx.symbol.Convolution(data=conv58_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_11_")
        # conv58_1_11_, output shape: {[16,14,14]}

        batchnorm58_1_11_ = mx.symbol.BatchNorm(data=conv58_1_11_,
            fix_gamma=True,
            name="batchnorm58_1_11_")
        relu58_1_11_ = mx.symbol.Activation(data=batchnorm58_1_11_,
            act_type='relu',
            name="relu58_1_11_")

        conv59_1_11_ = mx.symbol.Convolution(data=relu58_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_11_")
        # conv59_1_11_, output shape: {[1024,14,14]}

        batchnorm59_1_11_ = mx.symbol.BatchNorm(data=conv59_1_11_,
            fix_gamma=True,
            name="batchnorm59_1_11_")
        conv57_1_12_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_12_")
        # conv57_1_12_, output shape: {[16,14,14]}

        batchnorm57_1_12_ = mx.symbol.BatchNorm(data=conv57_1_12_,
            fix_gamma=True,
            name="batchnorm57_1_12_")
        relu57_1_12_ = mx.symbol.Activation(data=batchnorm57_1_12_,
            act_type='relu',
            name="relu57_1_12_")

        conv58_1_12_ = mx.symbol.pad(data=relu57_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_12_ = mx.symbol.Convolution(data=conv58_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_12_")
        # conv58_1_12_, output shape: {[16,14,14]}

        batchnorm58_1_12_ = mx.symbol.BatchNorm(data=conv58_1_12_,
            fix_gamma=True,
            name="batchnorm58_1_12_")
        relu58_1_12_ = mx.symbol.Activation(data=batchnorm58_1_12_,
            act_type='relu',
            name="relu58_1_12_")

        conv59_1_12_ = mx.symbol.Convolution(data=relu58_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_12_")
        # conv59_1_12_, output shape: {[1024,14,14]}

        batchnorm59_1_12_ = mx.symbol.BatchNorm(data=conv59_1_12_,
            fix_gamma=True,
            name="batchnorm59_1_12_")
        conv57_1_13_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_13_")
        # conv57_1_13_, output shape: {[16,14,14]}

        batchnorm57_1_13_ = mx.symbol.BatchNorm(data=conv57_1_13_,
            fix_gamma=True,
            name="batchnorm57_1_13_")
        relu57_1_13_ = mx.symbol.Activation(data=batchnorm57_1_13_,
            act_type='relu',
            name="relu57_1_13_")

        conv58_1_13_ = mx.symbol.pad(data=relu57_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_13_ = mx.symbol.Convolution(data=conv58_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_13_")
        # conv58_1_13_, output shape: {[16,14,14]}

        batchnorm58_1_13_ = mx.symbol.BatchNorm(data=conv58_1_13_,
            fix_gamma=True,
            name="batchnorm58_1_13_")
        relu58_1_13_ = mx.symbol.Activation(data=batchnorm58_1_13_,
            act_type='relu',
            name="relu58_1_13_")

        conv59_1_13_ = mx.symbol.Convolution(data=relu58_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_13_")
        # conv59_1_13_, output shape: {[1024,14,14]}

        batchnorm59_1_13_ = mx.symbol.BatchNorm(data=conv59_1_13_,
            fix_gamma=True,
            name="batchnorm59_1_13_")
        conv57_1_14_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_14_")
        # conv57_1_14_, output shape: {[16,14,14]}

        batchnorm57_1_14_ = mx.symbol.BatchNorm(data=conv57_1_14_,
            fix_gamma=True,
            name="batchnorm57_1_14_")
        relu57_1_14_ = mx.symbol.Activation(data=batchnorm57_1_14_,
            act_type='relu',
            name="relu57_1_14_")

        conv58_1_14_ = mx.symbol.pad(data=relu57_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_14_ = mx.symbol.Convolution(data=conv58_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_14_")
        # conv58_1_14_, output shape: {[16,14,14]}

        batchnorm58_1_14_ = mx.symbol.BatchNorm(data=conv58_1_14_,
            fix_gamma=True,
            name="batchnorm58_1_14_")
        relu58_1_14_ = mx.symbol.Activation(data=batchnorm58_1_14_,
            act_type='relu',
            name="relu58_1_14_")

        conv59_1_14_ = mx.symbol.Convolution(data=relu58_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_14_")
        # conv59_1_14_, output shape: {[1024,14,14]}

        batchnorm59_1_14_ = mx.symbol.BatchNorm(data=conv59_1_14_,
            fix_gamma=True,
            name="batchnorm59_1_14_")
        conv57_1_15_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_15_")
        # conv57_1_15_, output shape: {[16,14,14]}

        batchnorm57_1_15_ = mx.symbol.BatchNorm(data=conv57_1_15_,
            fix_gamma=True,
            name="batchnorm57_1_15_")
        relu57_1_15_ = mx.symbol.Activation(data=batchnorm57_1_15_,
            act_type='relu',
            name="relu57_1_15_")

        conv58_1_15_ = mx.symbol.pad(data=relu57_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_15_ = mx.symbol.Convolution(data=conv58_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_15_")
        # conv58_1_15_, output shape: {[16,14,14]}

        batchnorm58_1_15_ = mx.symbol.BatchNorm(data=conv58_1_15_,
            fix_gamma=True,
            name="batchnorm58_1_15_")
        relu58_1_15_ = mx.symbol.Activation(data=batchnorm58_1_15_,
            act_type='relu',
            name="relu58_1_15_")

        conv59_1_15_ = mx.symbol.Convolution(data=relu58_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_15_")
        # conv59_1_15_, output shape: {[1024,14,14]}

        batchnorm59_1_15_ = mx.symbol.BatchNorm(data=conv59_1_15_,
            fix_gamma=True,
            name="batchnorm59_1_15_")
        conv57_1_16_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_16_")
        # conv57_1_16_, output shape: {[16,14,14]}

        batchnorm57_1_16_ = mx.symbol.BatchNorm(data=conv57_1_16_,
            fix_gamma=True,
            name="batchnorm57_1_16_")
        relu57_1_16_ = mx.symbol.Activation(data=batchnorm57_1_16_,
            act_type='relu',
            name="relu57_1_16_")

        conv58_1_16_ = mx.symbol.pad(data=relu57_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_16_ = mx.symbol.Convolution(data=conv58_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_16_")
        # conv58_1_16_, output shape: {[16,14,14]}

        batchnorm58_1_16_ = mx.symbol.BatchNorm(data=conv58_1_16_,
            fix_gamma=True,
            name="batchnorm58_1_16_")
        relu58_1_16_ = mx.symbol.Activation(data=batchnorm58_1_16_,
            act_type='relu',
            name="relu58_1_16_")

        conv59_1_16_ = mx.symbol.Convolution(data=relu58_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_16_")
        # conv59_1_16_, output shape: {[1024,14,14]}

        batchnorm59_1_16_ = mx.symbol.BatchNorm(data=conv59_1_16_,
            fix_gamma=True,
            name="batchnorm59_1_16_")
        conv57_1_17_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_17_")
        # conv57_1_17_, output shape: {[16,14,14]}

        batchnorm57_1_17_ = mx.symbol.BatchNorm(data=conv57_1_17_,
            fix_gamma=True,
            name="batchnorm57_1_17_")
        relu57_1_17_ = mx.symbol.Activation(data=batchnorm57_1_17_,
            act_type='relu',
            name="relu57_1_17_")

        conv58_1_17_ = mx.symbol.pad(data=relu57_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_17_ = mx.symbol.Convolution(data=conv58_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_17_")
        # conv58_1_17_, output shape: {[16,14,14]}

        batchnorm58_1_17_ = mx.symbol.BatchNorm(data=conv58_1_17_,
            fix_gamma=True,
            name="batchnorm58_1_17_")
        relu58_1_17_ = mx.symbol.Activation(data=batchnorm58_1_17_,
            act_type='relu',
            name="relu58_1_17_")

        conv59_1_17_ = mx.symbol.Convolution(data=relu58_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_17_")
        # conv59_1_17_, output shape: {[1024,14,14]}

        batchnorm59_1_17_ = mx.symbol.BatchNorm(data=conv59_1_17_,
            fix_gamma=True,
            name="batchnorm59_1_17_")
        conv57_1_18_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_18_")
        # conv57_1_18_, output shape: {[16,14,14]}

        batchnorm57_1_18_ = mx.symbol.BatchNorm(data=conv57_1_18_,
            fix_gamma=True,
            name="batchnorm57_1_18_")
        relu57_1_18_ = mx.symbol.Activation(data=batchnorm57_1_18_,
            act_type='relu',
            name="relu57_1_18_")

        conv58_1_18_ = mx.symbol.pad(data=relu57_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_18_ = mx.symbol.Convolution(data=conv58_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_18_")
        # conv58_1_18_, output shape: {[16,14,14]}

        batchnorm58_1_18_ = mx.symbol.BatchNorm(data=conv58_1_18_,
            fix_gamma=True,
            name="batchnorm58_1_18_")
        relu58_1_18_ = mx.symbol.Activation(data=batchnorm58_1_18_,
            act_type='relu',
            name="relu58_1_18_")

        conv59_1_18_ = mx.symbol.Convolution(data=relu58_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_18_")
        # conv59_1_18_, output shape: {[1024,14,14]}

        batchnorm59_1_18_ = mx.symbol.BatchNorm(data=conv59_1_18_,
            fix_gamma=True,
            name="batchnorm59_1_18_")
        conv57_1_19_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_19_")
        # conv57_1_19_, output shape: {[16,14,14]}

        batchnorm57_1_19_ = mx.symbol.BatchNorm(data=conv57_1_19_,
            fix_gamma=True,
            name="batchnorm57_1_19_")
        relu57_1_19_ = mx.symbol.Activation(data=batchnorm57_1_19_,
            act_type='relu',
            name="relu57_1_19_")

        conv58_1_19_ = mx.symbol.pad(data=relu57_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_19_ = mx.symbol.Convolution(data=conv58_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_19_")
        # conv58_1_19_, output shape: {[16,14,14]}

        batchnorm58_1_19_ = mx.symbol.BatchNorm(data=conv58_1_19_,
            fix_gamma=True,
            name="batchnorm58_1_19_")
        relu58_1_19_ = mx.symbol.Activation(data=batchnorm58_1_19_,
            act_type='relu',
            name="relu58_1_19_")

        conv59_1_19_ = mx.symbol.Convolution(data=relu58_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_19_")
        # conv59_1_19_, output shape: {[1024,14,14]}

        batchnorm59_1_19_ = mx.symbol.BatchNorm(data=conv59_1_19_,
            fix_gamma=True,
            name="batchnorm59_1_19_")
        conv57_1_20_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_20_")
        # conv57_1_20_, output shape: {[16,14,14]}

        batchnorm57_1_20_ = mx.symbol.BatchNorm(data=conv57_1_20_,
            fix_gamma=True,
            name="batchnorm57_1_20_")
        relu57_1_20_ = mx.symbol.Activation(data=batchnorm57_1_20_,
            act_type='relu',
            name="relu57_1_20_")

        conv58_1_20_ = mx.symbol.pad(data=relu57_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_20_ = mx.symbol.Convolution(data=conv58_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_20_")
        # conv58_1_20_, output shape: {[16,14,14]}

        batchnorm58_1_20_ = mx.symbol.BatchNorm(data=conv58_1_20_,
            fix_gamma=True,
            name="batchnorm58_1_20_")
        relu58_1_20_ = mx.symbol.Activation(data=batchnorm58_1_20_,
            act_type='relu',
            name="relu58_1_20_")

        conv59_1_20_ = mx.symbol.Convolution(data=relu58_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_20_")
        # conv59_1_20_, output shape: {[1024,14,14]}

        batchnorm59_1_20_ = mx.symbol.BatchNorm(data=conv59_1_20_,
            fix_gamma=True,
            name="batchnorm59_1_20_")
        conv57_1_21_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_21_")
        # conv57_1_21_, output shape: {[16,14,14]}

        batchnorm57_1_21_ = mx.symbol.BatchNorm(data=conv57_1_21_,
            fix_gamma=True,
            name="batchnorm57_1_21_")
        relu57_1_21_ = mx.symbol.Activation(data=batchnorm57_1_21_,
            act_type='relu',
            name="relu57_1_21_")

        conv58_1_21_ = mx.symbol.pad(data=relu57_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_21_ = mx.symbol.Convolution(data=conv58_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_21_")
        # conv58_1_21_, output shape: {[16,14,14]}

        batchnorm58_1_21_ = mx.symbol.BatchNorm(data=conv58_1_21_,
            fix_gamma=True,
            name="batchnorm58_1_21_")
        relu58_1_21_ = mx.symbol.Activation(data=batchnorm58_1_21_,
            act_type='relu',
            name="relu58_1_21_")

        conv59_1_21_ = mx.symbol.Convolution(data=relu58_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_21_")
        # conv59_1_21_, output shape: {[1024,14,14]}

        batchnorm59_1_21_ = mx.symbol.BatchNorm(data=conv59_1_21_,
            fix_gamma=True,
            name="batchnorm59_1_21_")
        conv57_1_22_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_22_")
        # conv57_1_22_, output shape: {[16,14,14]}

        batchnorm57_1_22_ = mx.symbol.BatchNorm(data=conv57_1_22_,
            fix_gamma=True,
            name="batchnorm57_1_22_")
        relu57_1_22_ = mx.symbol.Activation(data=batchnorm57_1_22_,
            act_type='relu',
            name="relu57_1_22_")

        conv58_1_22_ = mx.symbol.pad(data=relu57_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_22_ = mx.symbol.Convolution(data=conv58_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_22_")
        # conv58_1_22_, output shape: {[16,14,14]}

        batchnorm58_1_22_ = mx.symbol.BatchNorm(data=conv58_1_22_,
            fix_gamma=True,
            name="batchnorm58_1_22_")
        relu58_1_22_ = mx.symbol.Activation(data=batchnorm58_1_22_,
            act_type='relu',
            name="relu58_1_22_")

        conv59_1_22_ = mx.symbol.Convolution(data=relu58_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_22_")
        # conv59_1_22_, output shape: {[1024,14,14]}

        batchnorm59_1_22_ = mx.symbol.BatchNorm(data=conv59_1_22_,
            fix_gamma=True,
            name="batchnorm59_1_22_")
        conv57_1_23_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_23_")
        # conv57_1_23_, output shape: {[16,14,14]}

        batchnorm57_1_23_ = mx.symbol.BatchNorm(data=conv57_1_23_,
            fix_gamma=True,
            name="batchnorm57_1_23_")
        relu57_1_23_ = mx.symbol.Activation(data=batchnorm57_1_23_,
            act_type='relu',
            name="relu57_1_23_")

        conv58_1_23_ = mx.symbol.pad(data=relu57_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_23_ = mx.symbol.Convolution(data=conv58_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_23_")
        # conv58_1_23_, output shape: {[16,14,14]}

        batchnorm58_1_23_ = mx.symbol.BatchNorm(data=conv58_1_23_,
            fix_gamma=True,
            name="batchnorm58_1_23_")
        relu58_1_23_ = mx.symbol.Activation(data=batchnorm58_1_23_,
            act_type='relu',
            name="relu58_1_23_")

        conv59_1_23_ = mx.symbol.Convolution(data=relu58_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_23_")
        # conv59_1_23_, output shape: {[1024,14,14]}

        batchnorm59_1_23_ = mx.symbol.BatchNorm(data=conv59_1_23_,
            fix_gamma=True,
            name="batchnorm59_1_23_")
        conv57_1_24_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_24_")
        # conv57_1_24_, output shape: {[16,14,14]}

        batchnorm57_1_24_ = mx.symbol.BatchNorm(data=conv57_1_24_,
            fix_gamma=True,
            name="batchnorm57_1_24_")
        relu57_1_24_ = mx.symbol.Activation(data=batchnorm57_1_24_,
            act_type='relu',
            name="relu57_1_24_")

        conv58_1_24_ = mx.symbol.pad(data=relu57_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_24_ = mx.symbol.Convolution(data=conv58_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_24_")
        # conv58_1_24_, output shape: {[16,14,14]}

        batchnorm58_1_24_ = mx.symbol.BatchNorm(data=conv58_1_24_,
            fix_gamma=True,
            name="batchnorm58_1_24_")
        relu58_1_24_ = mx.symbol.Activation(data=batchnorm58_1_24_,
            act_type='relu',
            name="relu58_1_24_")

        conv59_1_24_ = mx.symbol.Convolution(data=relu58_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_24_")
        # conv59_1_24_, output shape: {[1024,14,14]}

        batchnorm59_1_24_ = mx.symbol.BatchNorm(data=conv59_1_24_,
            fix_gamma=True,
            name="batchnorm59_1_24_")
        conv57_1_25_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_25_")
        # conv57_1_25_, output shape: {[16,14,14]}

        batchnorm57_1_25_ = mx.symbol.BatchNorm(data=conv57_1_25_,
            fix_gamma=True,
            name="batchnorm57_1_25_")
        relu57_1_25_ = mx.symbol.Activation(data=batchnorm57_1_25_,
            act_type='relu',
            name="relu57_1_25_")

        conv58_1_25_ = mx.symbol.pad(data=relu57_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_25_ = mx.symbol.Convolution(data=conv58_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_25_")
        # conv58_1_25_, output shape: {[16,14,14]}

        batchnorm58_1_25_ = mx.symbol.BatchNorm(data=conv58_1_25_,
            fix_gamma=True,
            name="batchnorm58_1_25_")
        relu58_1_25_ = mx.symbol.Activation(data=batchnorm58_1_25_,
            act_type='relu',
            name="relu58_1_25_")

        conv59_1_25_ = mx.symbol.Convolution(data=relu58_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_25_")
        # conv59_1_25_, output shape: {[1024,14,14]}

        batchnorm59_1_25_ = mx.symbol.BatchNorm(data=conv59_1_25_,
            fix_gamma=True,
            name="batchnorm59_1_25_")
        conv57_1_26_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_26_")
        # conv57_1_26_, output shape: {[16,14,14]}

        batchnorm57_1_26_ = mx.symbol.BatchNorm(data=conv57_1_26_,
            fix_gamma=True,
            name="batchnorm57_1_26_")
        relu57_1_26_ = mx.symbol.Activation(data=batchnorm57_1_26_,
            act_type='relu',
            name="relu57_1_26_")

        conv58_1_26_ = mx.symbol.pad(data=relu57_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_26_ = mx.symbol.Convolution(data=conv58_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_26_")
        # conv58_1_26_, output shape: {[16,14,14]}

        batchnorm58_1_26_ = mx.symbol.BatchNorm(data=conv58_1_26_,
            fix_gamma=True,
            name="batchnorm58_1_26_")
        relu58_1_26_ = mx.symbol.Activation(data=batchnorm58_1_26_,
            act_type='relu',
            name="relu58_1_26_")

        conv59_1_26_ = mx.symbol.Convolution(data=relu58_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_26_")
        # conv59_1_26_, output shape: {[1024,14,14]}

        batchnorm59_1_26_ = mx.symbol.BatchNorm(data=conv59_1_26_,
            fix_gamma=True,
            name="batchnorm59_1_26_")
        conv57_1_27_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_27_")
        # conv57_1_27_, output shape: {[16,14,14]}

        batchnorm57_1_27_ = mx.symbol.BatchNorm(data=conv57_1_27_,
            fix_gamma=True,
            name="batchnorm57_1_27_")
        relu57_1_27_ = mx.symbol.Activation(data=batchnorm57_1_27_,
            act_type='relu',
            name="relu57_1_27_")

        conv58_1_27_ = mx.symbol.pad(data=relu57_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_27_ = mx.symbol.Convolution(data=conv58_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_27_")
        # conv58_1_27_, output shape: {[16,14,14]}

        batchnorm58_1_27_ = mx.symbol.BatchNorm(data=conv58_1_27_,
            fix_gamma=True,
            name="batchnorm58_1_27_")
        relu58_1_27_ = mx.symbol.Activation(data=batchnorm58_1_27_,
            act_type='relu',
            name="relu58_1_27_")

        conv59_1_27_ = mx.symbol.Convolution(data=relu58_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_27_")
        # conv59_1_27_, output shape: {[1024,14,14]}

        batchnorm59_1_27_ = mx.symbol.BatchNorm(data=conv59_1_27_,
            fix_gamma=True,
            name="batchnorm59_1_27_")
        conv57_1_28_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_28_")
        # conv57_1_28_, output shape: {[16,14,14]}

        batchnorm57_1_28_ = mx.symbol.BatchNorm(data=conv57_1_28_,
            fix_gamma=True,
            name="batchnorm57_1_28_")
        relu57_1_28_ = mx.symbol.Activation(data=batchnorm57_1_28_,
            act_type='relu',
            name="relu57_1_28_")

        conv58_1_28_ = mx.symbol.pad(data=relu57_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_28_ = mx.symbol.Convolution(data=conv58_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_28_")
        # conv58_1_28_, output shape: {[16,14,14]}

        batchnorm58_1_28_ = mx.symbol.BatchNorm(data=conv58_1_28_,
            fix_gamma=True,
            name="batchnorm58_1_28_")
        relu58_1_28_ = mx.symbol.Activation(data=batchnorm58_1_28_,
            act_type='relu',
            name="relu58_1_28_")

        conv59_1_28_ = mx.symbol.Convolution(data=relu58_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_28_")
        # conv59_1_28_, output shape: {[1024,14,14]}

        batchnorm59_1_28_ = mx.symbol.BatchNorm(data=conv59_1_28_,
            fix_gamma=True,
            name="batchnorm59_1_28_")
        conv57_1_29_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_29_")
        # conv57_1_29_, output shape: {[16,14,14]}

        batchnorm57_1_29_ = mx.symbol.BatchNorm(data=conv57_1_29_,
            fix_gamma=True,
            name="batchnorm57_1_29_")
        relu57_1_29_ = mx.symbol.Activation(data=batchnorm57_1_29_,
            act_type='relu',
            name="relu57_1_29_")

        conv58_1_29_ = mx.symbol.pad(data=relu57_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_29_ = mx.symbol.Convolution(data=conv58_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_29_")
        # conv58_1_29_, output shape: {[16,14,14]}

        batchnorm58_1_29_ = mx.symbol.BatchNorm(data=conv58_1_29_,
            fix_gamma=True,
            name="batchnorm58_1_29_")
        relu58_1_29_ = mx.symbol.Activation(data=batchnorm58_1_29_,
            act_type='relu',
            name="relu58_1_29_")

        conv59_1_29_ = mx.symbol.Convolution(data=relu58_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_29_")
        # conv59_1_29_, output shape: {[1024,14,14]}

        batchnorm59_1_29_ = mx.symbol.BatchNorm(data=conv59_1_29_,
            fix_gamma=True,
            name="batchnorm59_1_29_")
        conv57_1_30_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_30_")
        # conv57_1_30_, output shape: {[16,14,14]}

        batchnorm57_1_30_ = mx.symbol.BatchNorm(data=conv57_1_30_,
            fix_gamma=True,
            name="batchnorm57_1_30_")
        relu57_1_30_ = mx.symbol.Activation(data=batchnorm57_1_30_,
            act_type='relu',
            name="relu57_1_30_")

        conv58_1_30_ = mx.symbol.pad(data=relu57_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_30_ = mx.symbol.Convolution(data=conv58_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_30_")
        # conv58_1_30_, output shape: {[16,14,14]}

        batchnorm58_1_30_ = mx.symbol.BatchNorm(data=conv58_1_30_,
            fix_gamma=True,
            name="batchnorm58_1_30_")
        relu58_1_30_ = mx.symbol.Activation(data=batchnorm58_1_30_,
            act_type='relu',
            name="relu58_1_30_")

        conv59_1_30_ = mx.symbol.Convolution(data=relu58_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_30_")
        # conv59_1_30_, output shape: {[1024,14,14]}

        batchnorm59_1_30_ = mx.symbol.BatchNorm(data=conv59_1_30_,
            fix_gamma=True,
            name="batchnorm59_1_30_")
        conv57_1_31_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_31_")
        # conv57_1_31_, output shape: {[16,14,14]}

        batchnorm57_1_31_ = mx.symbol.BatchNorm(data=conv57_1_31_,
            fix_gamma=True,
            name="batchnorm57_1_31_")
        relu57_1_31_ = mx.symbol.Activation(data=batchnorm57_1_31_,
            act_type='relu',
            name="relu57_1_31_")

        conv58_1_31_ = mx.symbol.pad(data=relu57_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_31_ = mx.symbol.Convolution(data=conv58_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_31_")
        # conv58_1_31_, output shape: {[16,14,14]}

        batchnorm58_1_31_ = mx.symbol.BatchNorm(data=conv58_1_31_,
            fix_gamma=True,
            name="batchnorm58_1_31_")
        relu58_1_31_ = mx.symbol.Activation(data=batchnorm58_1_31_,
            act_type='relu',
            name="relu58_1_31_")

        conv59_1_31_ = mx.symbol.Convolution(data=relu58_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_31_")
        # conv59_1_31_, output shape: {[1024,14,14]}

        batchnorm59_1_31_ = mx.symbol.BatchNorm(data=conv59_1_31_,
            fix_gamma=True,
            name="batchnorm59_1_31_")
        conv57_1_32_ = mx.symbol.Convolution(data=relu55_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv57_1_32_")
        # conv57_1_32_, output shape: {[16,14,14]}

        batchnorm57_1_32_ = mx.symbol.BatchNorm(data=conv57_1_32_,
            fix_gamma=True,
            name="batchnorm57_1_32_")
        relu57_1_32_ = mx.symbol.Activation(data=batchnorm57_1_32_,
            act_type='relu',
            name="relu57_1_32_")

        conv58_1_32_ = mx.symbol.pad(data=relu57_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv58_1_32_ = mx.symbol.Convolution(data=conv58_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv58_1_32_")
        # conv58_1_32_, output shape: {[16,14,14]}

        batchnorm58_1_32_ = mx.symbol.BatchNorm(data=conv58_1_32_,
            fix_gamma=True,
            name="batchnorm58_1_32_")
        relu58_1_32_ = mx.symbol.Activation(data=batchnorm58_1_32_,
            act_type='relu',
            name="relu58_1_32_")

        conv59_1_32_ = mx.symbol.Convolution(data=relu58_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv59_1_32_")
        # conv59_1_32_, output shape: {[1024,14,14]}

        batchnorm59_1_32_ = mx.symbol.BatchNorm(data=conv59_1_32_,
            fix_gamma=True,
            name="batchnorm59_1_32_")
        add60_1_ = batchnorm59_1_1_ + batchnorm59_1_2_ + batchnorm59_1_3_ + batchnorm59_1_4_ + batchnorm59_1_5_ + batchnorm59_1_6_ + batchnorm59_1_7_ + batchnorm59_1_8_ + batchnorm59_1_9_ + batchnorm59_1_10_ + batchnorm59_1_11_ + batchnorm59_1_12_ + batchnorm59_1_13_ + batchnorm59_1_14_ + batchnorm59_1_15_ + batchnorm59_1_16_ + batchnorm59_1_17_ + batchnorm59_1_18_ + batchnorm59_1_19_ + batchnorm59_1_20_ + batchnorm59_1_21_ + batchnorm59_1_22_ + batchnorm59_1_23_ + batchnorm59_1_24_ + batchnorm59_1_25_ + batchnorm59_1_26_ + batchnorm59_1_27_ + batchnorm59_1_28_ + batchnorm59_1_29_ + batchnorm59_1_30_ + batchnorm59_1_31_ + batchnorm59_1_32_
        # add60_1_, output shape: {[1024,14,14]}

        add61_ = add60_1_ + relu55_
        # add61_, output shape: {[1024,14,14]}

        relu61_ = mx.symbol.Activation(data=add61_,
            act_type='relu',
            name="relu61_")

        conv63_1_1_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_1_")
        # conv63_1_1_, output shape: {[16,14,14]}

        batchnorm63_1_1_ = mx.symbol.BatchNorm(data=conv63_1_1_,
            fix_gamma=True,
            name="batchnorm63_1_1_")
        relu63_1_1_ = mx.symbol.Activation(data=batchnorm63_1_1_,
            act_type='relu',
            name="relu63_1_1_")

        conv64_1_1_ = mx.symbol.pad(data=relu63_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_1_ = mx.symbol.Convolution(data=conv64_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_1_")
        # conv64_1_1_, output shape: {[16,14,14]}

        batchnorm64_1_1_ = mx.symbol.BatchNorm(data=conv64_1_1_,
            fix_gamma=True,
            name="batchnorm64_1_1_")
        relu64_1_1_ = mx.symbol.Activation(data=batchnorm64_1_1_,
            act_type='relu',
            name="relu64_1_1_")

        conv65_1_1_ = mx.symbol.Convolution(data=relu64_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_1_")
        # conv65_1_1_, output shape: {[1024,14,14]}

        batchnorm65_1_1_ = mx.symbol.BatchNorm(data=conv65_1_1_,
            fix_gamma=True,
            name="batchnorm65_1_1_")
        conv63_1_2_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_2_")
        # conv63_1_2_, output shape: {[16,14,14]}

        batchnorm63_1_2_ = mx.symbol.BatchNorm(data=conv63_1_2_,
            fix_gamma=True,
            name="batchnorm63_1_2_")
        relu63_1_2_ = mx.symbol.Activation(data=batchnorm63_1_2_,
            act_type='relu',
            name="relu63_1_2_")

        conv64_1_2_ = mx.symbol.pad(data=relu63_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_2_ = mx.symbol.Convolution(data=conv64_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_2_")
        # conv64_1_2_, output shape: {[16,14,14]}

        batchnorm64_1_2_ = mx.symbol.BatchNorm(data=conv64_1_2_,
            fix_gamma=True,
            name="batchnorm64_1_2_")
        relu64_1_2_ = mx.symbol.Activation(data=batchnorm64_1_2_,
            act_type='relu',
            name="relu64_1_2_")

        conv65_1_2_ = mx.symbol.Convolution(data=relu64_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_2_")
        # conv65_1_2_, output shape: {[1024,14,14]}

        batchnorm65_1_2_ = mx.symbol.BatchNorm(data=conv65_1_2_,
            fix_gamma=True,
            name="batchnorm65_1_2_")
        conv63_1_3_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_3_")
        # conv63_1_3_, output shape: {[16,14,14]}

        batchnorm63_1_3_ = mx.symbol.BatchNorm(data=conv63_1_3_,
            fix_gamma=True,
            name="batchnorm63_1_3_")
        relu63_1_3_ = mx.symbol.Activation(data=batchnorm63_1_3_,
            act_type='relu',
            name="relu63_1_3_")

        conv64_1_3_ = mx.symbol.pad(data=relu63_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_3_ = mx.symbol.Convolution(data=conv64_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_3_")
        # conv64_1_3_, output shape: {[16,14,14]}

        batchnorm64_1_3_ = mx.symbol.BatchNorm(data=conv64_1_3_,
            fix_gamma=True,
            name="batchnorm64_1_3_")
        relu64_1_3_ = mx.symbol.Activation(data=batchnorm64_1_3_,
            act_type='relu',
            name="relu64_1_3_")

        conv65_1_3_ = mx.symbol.Convolution(data=relu64_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_3_")
        # conv65_1_3_, output shape: {[1024,14,14]}

        batchnorm65_1_3_ = mx.symbol.BatchNorm(data=conv65_1_3_,
            fix_gamma=True,
            name="batchnorm65_1_3_")
        conv63_1_4_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_4_")
        # conv63_1_4_, output shape: {[16,14,14]}

        batchnorm63_1_4_ = mx.symbol.BatchNorm(data=conv63_1_4_,
            fix_gamma=True,
            name="batchnorm63_1_4_")
        relu63_1_4_ = mx.symbol.Activation(data=batchnorm63_1_4_,
            act_type='relu',
            name="relu63_1_4_")

        conv64_1_4_ = mx.symbol.pad(data=relu63_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_4_ = mx.symbol.Convolution(data=conv64_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_4_")
        # conv64_1_4_, output shape: {[16,14,14]}

        batchnorm64_1_4_ = mx.symbol.BatchNorm(data=conv64_1_4_,
            fix_gamma=True,
            name="batchnorm64_1_4_")
        relu64_1_4_ = mx.symbol.Activation(data=batchnorm64_1_4_,
            act_type='relu',
            name="relu64_1_4_")

        conv65_1_4_ = mx.symbol.Convolution(data=relu64_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_4_")
        # conv65_1_4_, output shape: {[1024,14,14]}

        batchnorm65_1_4_ = mx.symbol.BatchNorm(data=conv65_1_4_,
            fix_gamma=True,
            name="batchnorm65_1_4_")
        conv63_1_5_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_5_")
        # conv63_1_5_, output shape: {[16,14,14]}

        batchnorm63_1_5_ = mx.symbol.BatchNorm(data=conv63_1_5_,
            fix_gamma=True,
            name="batchnorm63_1_5_")
        relu63_1_5_ = mx.symbol.Activation(data=batchnorm63_1_5_,
            act_type='relu',
            name="relu63_1_5_")

        conv64_1_5_ = mx.symbol.pad(data=relu63_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_5_ = mx.symbol.Convolution(data=conv64_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_5_")
        # conv64_1_5_, output shape: {[16,14,14]}

        batchnorm64_1_5_ = mx.symbol.BatchNorm(data=conv64_1_5_,
            fix_gamma=True,
            name="batchnorm64_1_5_")
        relu64_1_5_ = mx.symbol.Activation(data=batchnorm64_1_5_,
            act_type='relu',
            name="relu64_1_5_")

        conv65_1_5_ = mx.symbol.Convolution(data=relu64_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_5_")
        # conv65_1_5_, output shape: {[1024,14,14]}

        batchnorm65_1_5_ = mx.symbol.BatchNorm(data=conv65_1_5_,
            fix_gamma=True,
            name="batchnorm65_1_5_")
        conv63_1_6_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_6_")
        # conv63_1_6_, output shape: {[16,14,14]}

        batchnorm63_1_6_ = mx.symbol.BatchNorm(data=conv63_1_6_,
            fix_gamma=True,
            name="batchnorm63_1_6_")
        relu63_1_6_ = mx.symbol.Activation(data=batchnorm63_1_6_,
            act_type='relu',
            name="relu63_1_6_")

        conv64_1_6_ = mx.symbol.pad(data=relu63_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_6_ = mx.symbol.Convolution(data=conv64_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_6_")
        # conv64_1_6_, output shape: {[16,14,14]}

        batchnorm64_1_6_ = mx.symbol.BatchNorm(data=conv64_1_6_,
            fix_gamma=True,
            name="batchnorm64_1_6_")
        relu64_1_6_ = mx.symbol.Activation(data=batchnorm64_1_6_,
            act_type='relu',
            name="relu64_1_6_")

        conv65_1_6_ = mx.symbol.Convolution(data=relu64_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_6_")
        # conv65_1_6_, output shape: {[1024,14,14]}

        batchnorm65_1_6_ = mx.symbol.BatchNorm(data=conv65_1_6_,
            fix_gamma=True,
            name="batchnorm65_1_6_")
        conv63_1_7_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_7_")
        # conv63_1_7_, output shape: {[16,14,14]}

        batchnorm63_1_7_ = mx.symbol.BatchNorm(data=conv63_1_7_,
            fix_gamma=True,
            name="batchnorm63_1_7_")
        relu63_1_7_ = mx.symbol.Activation(data=batchnorm63_1_7_,
            act_type='relu',
            name="relu63_1_7_")

        conv64_1_7_ = mx.symbol.pad(data=relu63_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_7_ = mx.symbol.Convolution(data=conv64_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_7_")
        # conv64_1_7_, output shape: {[16,14,14]}

        batchnorm64_1_7_ = mx.symbol.BatchNorm(data=conv64_1_7_,
            fix_gamma=True,
            name="batchnorm64_1_7_")
        relu64_1_7_ = mx.symbol.Activation(data=batchnorm64_1_7_,
            act_type='relu',
            name="relu64_1_7_")

        conv65_1_7_ = mx.symbol.Convolution(data=relu64_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_7_")
        # conv65_1_7_, output shape: {[1024,14,14]}

        batchnorm65_1_7_ = mx.symbol.BatchNorm(data=conv65_1_7_,
            fix_gamma=True,
            name="batchnorm65_1_7_")
        conv63_1_8_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_8_")
        # conv63_1_8_, output shape: {[16,14,14]}

        batchnorm63_1_8_ = mx.symbol.BatchNorm(data=conv63_1_8_,
            fix_gamma=True,
            name="batchnorm63_1_8_")
        relu63_1_8_ = mx.symbol.Activation(data=batchnorm63_1_8_,
            act_type='relu',
            name="relu63_1_8_")

        conv64_1_8_ = mx.symbol.pad(data=relu63_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_8_ = mx.symbol.Convolution(data=conv64_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_8_")
        # conv64_1_8_, output shape: {[16,14,14]}

        batchnorm64_1_8_ = mx.symbol.BatchNorm(data=conv64_1_8_,
            fix_gamma=True,
            name="batchnorm64_1_8_")
        relu64_1_8_ = mx.symbol.Activation(data=batchnorm64_1_8_,
            act_type='relu',
            name="relu64_1_8_")

        conv65_1_8_ = mx.symbol.Convolution(data=relu64_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_8_")
        # conv65_1_8_, output shape: {[1024,14,14]}

        batchnorm65_1_8_ = mx.symbol.BatchNorm(data=conv65_1_8_,
            fix_gamma=True,
            name="batchnorm65_1_8_")
        conv63_1_9_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_9_")
        # conv63_1_9_, output shape: {[16,14,14]}

        batchnorm63_1_9_ = mx.symbol.BatchNorm(data=conv63_1_9_,
            fix_gamma=True,
            name="batchnorm63_1_9_")
        relu63_1_9_ = mx.symbol.Activation(data=batchnorm63_1_9_,
            act_type='relu',
            name="relu63_1_9_")

        conv64_1_9_ = mx.symbol.pad(data=relu63_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_9_ = mx.symbol.Convolution(data=conv64_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_9_")
        # conv64_1_9_, output shape: {[16,14,14]}

        batchnorm64_1_9_ = mx.symbol.BatchNorm(data=conv64_1_9_,
            fix_gamma=True,
            name="batchnorm64_1_9_")
        relu64_1_9_ = mx.symbol.Activation(data=batchnorm64_1_9_,
            act_type='relu',
            name="relu64_1_9_")

        conv65_1_9_ = mx.symbol.Convolution(data=relu64_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_9_")
        # conv65_1_9_, output shape: {[1024,14,14]}

        batchnorm65_1_9_ = mx.symbol.BatchNorm(data=conv65_1_9_,
            fix_gamma=True,
            name="batchnorm65_1_9_")
        conv63_1_10_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_10_")
        # conv63_1_10_, output shape: {[16,14,14]}

        batchnorm63_1_10_ = mx.symbol.BatchNorm(data=conv63_1_10_,
            fix_gamma=True,
            name="batchnorm63_1_10_")
        relu63_1_10_ = mx.symbol.Activation(data=batchnorm63_1_10_,
            act_type='relu',
            name="relu63_1_10_")

        conv64_1_10_ = mx.symbol.pad(data=relu63_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_10_ = mx.symbol.Convolution(data=conv64_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_10_")
        # conv64_1_10_, output shape: {[16,14,14]}

        batchnorm64_1_10_ = mx.symbol.BatchNorm(data=conv64_1_10_,
            fix_gamma=True,
            name="batchnorm64_1_10_")
        relu64_1_10_ = mx.symbol.Activation(data=batchnorm64_1_10_,
            act_type='relu',
            name="relu64_1_10_")

        conv65_1_10_ = mx.symbol.Convolution(data=relu64_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_10_")
        # conv65_1_10_, output shape: {[1024,14,14]}

        batchnorm65_1_10_ = mx.symbol.BatchNorm(data=conv65_1_10_,
            fix_gamma=True,
            name="batchnorm65_1_10_")
        conv63_1_11_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_11_")
        # conv63_1_11_, output shape: {[16,14,14]}

        batchnorm63_1_11_ = mx.symbol.BatchNorm(data=conv63_1_11_,
            fix_gamma=True,
            name="batchnorm63_1_11_")
        relu63_1_11_ = mx.symbol.Activation(data=batchnorm63_1_11_,
            act_type='relu',
            name="relu63_1_11_")

        conv64_1_11_ = mx.symbol.pad(data=relu63_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_11_ = mx.symbol.Convolution(data=conv64_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_11_")
        # conv64_1_11_, output shape: {[16,14,14]}

        batchnorm64_1_11_ = mx.symbol.BatchNorm(data=conv64_1_11_,
            fix_gamma=True,
            name="batchnorm64_1_11_")
        relu64_1_11_ = mx.symbol.Activation(data=batchnorm64_1_11_,
            act_type='relu',
            name="relu64_1_11_")

        conv65_1_11_ = mx.symbol.Convolution(data=relu64_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_11_")
        # conv65_1_11_, output shape: {[1024,14,14]}

        batchnorm65_1_11_ = mx.symbol.BatchNorm(data=conv65_1_11_,
            fix_gamma=True,
            name="batchnorm65_1_11_")
        conv63_1_12_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_12_")
        # conv63_1_12_, output shape: {[16,14,14]}

        batchnorm63_1_12_ = mx.symbol.BatchNorm(data=conv63_1_12_,
            fix_gamma=True,
            name="batchnorm63_1_12_")
        relu63_1_12_ = mx.symbol.Activation(data=batchnorm63_1_12_,
            act_type='relu',
            name="relu63_1_12_")

        conv64_1_12_ = mx.symbol.pad(data=relu63_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_12_ = mx.symbol.Convolution(data=conv64_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_12_")
        # conv64_1_12_, output shape: {[16,14,14]}

        batchnorm64_1_12_ = mx.symbol.BatchNorm(data=conv64_1_12_,
            fix_gamma=True,
            name="batchnorm64_1_12_")
        relu64_1_12_ = mx.symbol.Activation(data=batchnorm64_1_12_,
            act_type='relu',
            name="relu64_1_12_")

        conv65_1_12_ = mx.symbol.Convolution(data=relu64_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_12_")
        # conv65_1_12_, output shape: {[1024,14,14]}

        batchnorm65_1_12_ = mx.symbol.BatchNorm(data=conv65_1_12_,
            fix_gamma=True,
            name="batchnorm65_1_12_")
        conv63_1_13_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_13_")
        # conv63_1_13_, output shape: {[16,14,14]}

        batchnorm63_1_13_ = mx.symbol.BatchNorm(data=conv63_1_13_,
            fix_gamma=True,
            name="batchnorm63_1_13_")
        relu63_1_13_ = mx.symbol.Activation(data=batchnorm63_1_13_,
            act_type='relu',
            name="relu63_1_13_")

        conv64_1_13_ = mx.symbol.pad(data=relu63_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_13_ = mx.symbol.Convolution(data=conv64_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_13_")
        # conv64_1_13_, output shape: {[16,14,14]}

        batchnorm64_1_13_ = mx.symbol.BatchNorm(data=conv64_1_13_,
            fix_gamma=True,
            name="batchnorm64_1_13_")
        relu64_1_13_ = mx.symbol.Activation(data=batchnorm64_1_13_,
            act_type='relu',
            name="relu64_1_13_")

        conv65_1_13_ = mx.symbol.Convolution(data=relu64_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_13_")
        # conv65_1_13_, output shape: {[1024,14,14]}

        batchnorm65_1_13_ = mx.symbol.BatchNorm(data=conv65_1_13_,
            fix_gamma=True,
            name="batchnorm65_1_13_")
        conv63_1_14_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_14_")
        # conv63_1_14_, output shape: {[16,14,14]}

        batchnorm63_1_14_ = mx.symbol.BatchNorm(data=conv63_1_14_,
            fix_gamma=True,
            name="batchnorm63_1_14_")
        relu63_1_14_ = mx.symbol.Activation(data=batchnorm63_1_14_,
            act_type='relu',
            name="relu63_1_14_")

        conv64_1_14_ = mx.symbol.pad(data=relu63_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_14_ = mx.symbol.Convolution(data=conv64_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_14_")
        # conv64_1_14_, output shape: {[16,14,14]}

        batchnorm64_1_14_ = mx.symbol.BatchNorm(data=conv64_1_14_,
            fix_gamma=True,
            name="batchnorm64_1_14_")
        relu64_1_14_ = mx.symbol.Activation(data=batchnorm64_1_14_,
            act_type='relu',
            name="relu64_1_14_")

        conv65_1_14_ = mx.symbol.Convolution(data=relu64_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_14_")
        # conv65_1_14_, output shape: {[1024,14,14]}

        batchnorm65_1_14_ = mx.symbol.BatchNorm(data=conv65_1_14_,
            fix_gamma=True,
            name="batchnorm65_1_14_")
        conv63_1_15_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_15_")
        # conv63_1_15_, output shape: {[16,14,14]}

        batchnorm63_1_15_ = mx.symbol.BatchNorm(data=conv63_1_15_,
            fix_gamma=True,
            name="batchnorm63_1_15_")
        relu63_1_15_ = mx.symbol.Activation(data=batchnorm63_1_15_,
            act_type='relu',
            name="relu63_1_15_")

        conv64_1_15_ = mx.symbol.pad(data=relu63_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_15_ = mx.symbol.Convolution(data=conv64_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_15_")
        # conv64_1_15_, output shape: {[16,14,14]}

        batchnorm64_1_15_ = mx.symbol.BatchNorm(data=conv64_1_15_,
            fix_gamma=True,
            name="batchnorm64_1_15_")
        relu64_1_15_ = mx.symbol.Activation(data=batchnorm64_1_15_,
            act_type='relu',
            name="relu64_1_15_")

        conv65_1_15_ = mx.symbol.Convolution(data=relu64_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_15_")
        # conv65_1_15_, output shape: {[1024,14,14]}

        batchnorm65_1_15_ = mx.symbol.BatchNorm(data=conv65_1_15_,
            fix_gamma=True,
            name="batchnorm65_1_15_")
        conv63_1_16_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_16_")
        # conv63_1_16_, output shape: {[16,14,14]}

        batchnorm63_1_16_ = mx.symbol.BatchNorm(data=conv63_1_16_,
            fix_gamma=True,
            name="batchnorm63_1_16_")
        relu63_1_16_ = mx.symbol.Activation(data=batchnorm63_1_16_,
            act_type='relu',
            name="relu63_1_16_")

        conv64_1_16_ = mx.symbol.pad(data=relu63_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_16_ = mx.symbol.Convolution(data=conv64_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_16_")
        # conv64_1_16_, output shape: {[16,14,14]}

        batchnorm64_1_16_ = mx.symbol.BatchNorm(data=conv64_1_16_,
            fix_gamma=True,
            name="batchnorm64_1_16_")
        relu64_1_16_ = mx.symbol.Activation(data=batchnorm64_1_16_,
            act_type='relu',
            name="relu64_1_16_")

        conv65_1_16_ = mx.symbol.Convolution(data=relu64_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_16_")
        # conv65_1_16_, output shape: {[1024,14,14]}

        batchnorm65_1_16_ = mx.symbol.BatchNorm(data=conv65_1_16_,
            fix_gamma=True,
            name="batchnorm65_1_16_")
        conv63_1_17_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_17_")
        # conv63_1_17_, output shape: {[16,14,14]}

        batchnorm63_1_17_ = mx.symbol.BatchNorm(data=conv63_1_17_,
            fix_gamma=True,
            name="batchnorm63_1_17_")
        relu63_1_17_ = mx.symbol.Activation(data=batchnorm63_1_17_,
            act_type='relu',
            name="relu63_1_17_")

        conv64_1_17_ = mx.symbol.pad(data=relu63_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_17_ = mx.symbol.Convolution(data=conv64_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_17_")
        # conv64_1_17_, output shape: {[16,14,14]}

        batchnorm64_1_17_ = mx.symbol.BatchNorm(data=conv64_1_17_,
            fix_gamma=True,
            name="batchnorm64_1_17_")
        relu64_1_17_ = mx.symbol.Activation(data=batchnorm64_1_17_,
            act_type='relu',
            name="relu64_1_17_")

        conv65_1_17_ = mx.symbol.Convolution(data=relu64_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_17_")
        # conv65_1_17_, output shape: {[1024,14,14]}

        batchnorm65_1_17_ = mx.symbol.BatchNorm(data=conv65_1_17_,
            fix_gamma=True,
            name="batchnorm65_1_17_")
        conv63_1_18_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_18_")
        # conv63_1_18_, output shape: {[16,14,14]}

        batchnorm63_1_18_ = mx.symbol.BatchNorm(data=conv63_1_18_,
            fix_gamma=True,
            name="batchnorm63_1_18_")
        relu63_1_18_ = mx.symbol.Activation(data=batchnorm63_1_18_,
            act_type='relu',
            name="relu63_1_18_")

        conv64_1_18_ = mx.symbol.pad(data=relu63_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_18_ = mx.symbol.Convolution(data=conv64_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_18_")
        # conv64_1_18_, output shape: {[16,14,14]}

        batchnorm64_1_18_ = mx.symbol.BatchNorm(data=conv64_1_18_,
            fix_gamma=True,
            name="batchnorm64_1_18_")
        relu64_1_18_ = mx.symbol.Activation(data=batchnorm64_1_18_,
            act_type='relu',
            name="relu64_1_18_")

        conv65_1_18_ = mx.symbol.Convolution(data=relu64_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_18_")
        # conv65_1_18_, output shape: {[1024,14,14]}

        batchnorm65_1_18_ = mx.symbol.BatchNorm(data=conv65_1_18_,
            fix_gamma=True,
            name="batchnorm65_1_18_")
        conv63_1_19_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_19_")
        # conv63_1_19_, output shape: {[16,14,14]}

        batchnorm63_1_19_ = mx.symbol.BatchNorm(data=conv63_1_19_,
            fix_gamma=True,
            name="batchnorm63_1_19_")
        relu63_1_19_ = mx.symbol.Activation(data=batchnorm63_1_19_,
            act_type='relu',
            name="relu63_1_19_")

        conv64_1_19_ = mx.symbol.pad(data=relu63_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_19_ = mx.symbol.Convolution(data=conv64_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_19_")
        # conv64_1_19_, output shape: {[16,14,14]}

        batchnorm64_1_19_ = mx.symbol.BatchNorm(data=conv64_1_19_,
            fix_gamma=True,
            name="batchnorm64_1_19_")
        relu64_1_19_ = mx.symbol.Activation(data=batchnorm64_1_19_,
            act_type='relu',
            name="relu64_1_19_")

        conv65_1_19_ = mx.symbol.Convolution(data=relu64_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_19_")
        # conv65_1_19_, output shape: {[1024,14,14]}

        batchnorm65_1_19_ = mx.symbol.BatchNorm(data=conv65_1_19_,
            fix_gamma=True,
            name="batchnorm65_1_19_")
        conv63_1_20_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_20_")
        # conv63_1_20_, output shape: {[16,14,14]}

        batchnorm63_1_20_ = mx.symbol.BatchNorm(data=conv63_1_20_,
            fix_gamma=True,
            name="batchnorm63_1_20_")
        relu63_1_20_ = mx.symbol.Activation(data=batchnorm63_1_20_,
            act_type='relu',
            name="relu63_1_20_")

        conv64_1_20_ = mx.symbol.pad(data=relu63_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_20_ = mx.symbol.Convolution(data=conv64_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_20_")
        # conv64_1_20_, output shape: {[16,14,14]}

        batchnorm64_1_20_ = mx.symbol.BatchNorm(data=conv64_1_20_,
            fix_gamma=True,
            name="batchnorm64_1_20_")
        relu64_1_20_ = mx.symbol.Activation(data=batchnorm64_1_20_,
            act_type='relu',
            name="relu64_1_20_")

        conv65_1_20_ = mx.symbol.Convolution(data=relu64_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_20_")
        # conv65_1_20_, output shape: {[1024,14,14]}

        batchnorm65_1_20_ = mx.symbol.BatchNorm(data=conv65_1_20_,
            fix_gamma=True,
            name="batchnorm65_1_20_")
        conv63_1_21_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_21_")
        # conv63_1_21_, output shape: {[16,14,14]}

        batchnorm63_1_21_ = mx.symbol.BatchNorm(data=conv63_1_21_,
            fix_gamma=True,
            name="batchnorm63_1_21_")
        relu63_1_21_ = mx.symbol.Activation(data=batchnorm63_1_21_,
            act_type='relu',
            name="relu63_1_21_")

        conv64_1_21_ = mx.symbol.pad(data=relu63_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_21_ = mx.symbol.Convolution(data=conv64_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_21_")
        # conv64_1_21_, output shape: {[16,14,14]}

        batchnorm64_1_21_ = mx.symbol.BatchNorm(data=conv64_1_21_,
            fix_gamma=True,
            name="batchnorm64_1_21_")
        relu64_1_21_ = mx.symbol.Activation(data=batchnorm64_1_21_,
            act_type='relu',
            name="relu64_1_21_")

        conv65_1_21_ = mx.symbol.Convolution(data=relu64_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_21_")
        # conv65_1_21_, output shape: {[1024,14,14]}

        batchnorm65_1_21_ = mx.symbol.BatchNorm(data=conv65_1_21_,
            fix_gamma=True,
            name="batchnorm65_1_21_")
        conv63_1_22_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_22_")
        # conv63_1_22_, output shape: {[16,14,14]}

        batchnorm63_1_22_ = mx.symbol.BatchNorm(data=conv63_1_22_,
            fix_gamma=True,
            name="batchnorm63_1_22_")
        relu63_1_22_ = mx.symbol.Activation(data=batchnorm63_1_22_,
            act_type='relu',
            name="relu63_1_22_")

        conv64_1_22_ = mx.symbol.pad(data=relu63_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_22_ = mx.symbol.Convolution(data=conv64_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_22_")
        # conv64_1_22_, output shape: {[16,14,14]}

        batchnorm64_1_22_ = mx.symbol.BatchNorm(data=conv64_1_22_,
            fix_gamma=True,
            name="batchnorm64_1_22_")
        relu64_1_22_ = mx.symbol.Activation(data=batchnorm64_1_22_,
            act_type='relu',
            name="relu64_1_22_")

        conv65_1_22_ = mx.symbol.Convolution(data=relu64_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_22_")
        # conv65_1_22_, output shape: {[1024,14,14]}

        batchnorm65_1_22_ = mx.symbol.BatchNorm(data=conv65_1_22_,
            fix_gamma=True,
            name="batchnorm65_1_22_")
        conv63_1_23_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_23_")
        # conv63_1_23_, output shape: {[16,14,14]}

        batchnorm63_1_23_ = mx.symbol.BatchNorm(data=conv63_1_23_,
            fix_gamma=True,
            name="batchnorm63_1_23_")
        relu63_1_23_ = mx.symbol.Activation(data=batchnorm63_1_23_,
            act_type='relu',
            name="relu63_1_23_")

        conv64_1_23_ = mx.symbol.pad(data=relu63_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_23_ = mx.symbol.Convolution(data=conv64_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_23_")
        # conv64_1_23_, output shape: {[16,14,14]}

        batchnorm64_1_23_ = mx.symbol.BatchNorm(data=conv64_1_23_,
            fix_gamma=True,
            name="batchnorm64_1_23_")
        relu64_1_23_ = mx.symbol.Activation(data=batchnorm64_1_23_,
            act_type='relu',
            name="relu64_1_23_")

        conv65_1_23_ = mx.symbol.Convolution(data=relu64_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_23_")
        # conv65_1_23_, output shape: {[1024,14,14]}

        batchnorm65_1_23_ = mx.symbol.BatchNorm(data=conv65_1_23_,
            fix_gamma=True,
            name="batchnorm65_1_23_")
        conv63_1_24_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_24_")
        # conv63_1_24_, output shape: {[16,14,14]}

        batchnorm63_1_24_ = mx.symbol.BatchNorm(data=conv63_1_24_,
            fix_gamma=True,
            name="batchnorm63_1_24_")
        relu63_1_24_ = mx.symbol.Activation(data=batchnorm63_1_24_,
            act_type='relu',
            name="relu63_1_24_")

        conv64_1_24_ = mx.symbol.pad(data=relu63_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_24_ = mx.symbol.Convolution(data=conv64_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_24_")
        # conv64_1_24_, output shape: {[16,14,14]}

        batchnorm64_1_24_ = mx.symbol.BatchNorm(data=conv64_1_24_,
            fix_gamma=True,
            name="batchnorm64_1_24_")
        relu64_1_24_ = mx.symbol.Activation(data=batchnorm64_1_24_,
            act_type='relu',
            name="relu64_1_24_")

        conv65_1_24_ = mx.symbol.Convolution(data=relu64_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_24_")
        # conv65_1_24_, output shape: {[1024,14,14]}

        batchnorm65_1_24_ = mx.symbol.BatchNorm(data=conv65_1_24_,
            fix_gamma=True,
            name="batchnorm65_1_24_")
        conv63_1_25_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_25_")
        # conv63_1_25_, output shape: {[16,14,14]}

        batchnorm63_1_25_ = mx.symbol.BatchNorm(data=conv63_1_25_,
            fix_gamma=True,
            name="batchnorm63_1_25_")
        relu63_1_25_ = mx.symbol.Activation(data=batchnorm63_1_25_,
            act_type='relu',
            name="relu63_1_25_")

        conv64_1_25_ = mx.symbol.pad(data=relu63_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_25_ = mx.symbol.Convolution(data=conv64_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_25_")
        # conv64_1_25_, output shape: {[16,14,14]}

        batchnorm64_1_25_ = mx.symbol.BatchNorm(data=conv64_1_25_,
            fix_gamma=True,
            name="batchnorm64_1_25_")
        relu64_1_25_ = mx.symbol.Activation(data=batchnorm64_1_25_,
            act_type='relu',
            name="relu64_1_25_")

        conv65_1_25_ = mx.symbol.Convolution(data=relu64_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_25_")
        # conv65_1_25_, output shape: {[1024,14,14]}

        batchnorm65_1_25_ = mx.symbol.BatchNorm(data=conv65_1_25_,
            fix_gamma=True,
            name="batchnorm65_1_25_")
        conv63_1_26_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_26_")
        # conv63_1_26_, output shape: {[16,14,14]}

        batchnorm63_1_26_ = mx.symbol.BatchNorm(data=conv63_1_26_,
            fix_gamma=True,
            name="batchnorm63_1_26_")
        relu63_1_26_ = mx.symbol.Activation(data=batchnorm63_1_26_,
            act_type='relu',
            name="relu63_1_26_")

        conv64_1_26_ = mx.symbol.pad(data=relu63_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_26_ = mx.symbol.Convolution(data=conv64_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_26_")
        # conv64_1_26_, output shape: {[16,14,14]}

        batchnorm64_1_26_ = mx.symbol.BatchNorm(data=conv64_1_26_,
            fix_gamma=True,
            name="batchnorm64_1_26_")
        relu64_1_26_ = mx.symbol.Activation(data=batchnorm64_1_26_,
            act_type='relu',
            name="relu64_1_26_")

        conv65_1_26_ = mx.symbol.Convolution(data=relu64_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_26_")
        # conv65_1_26_, output shape: {[1024,14,14]}

        batchnorm65_1_26_ = mx.symbol.BatchNorm(data=conv65_1_26_,
            fix_gamma=True,
            name="batchnorm65_1_26_")
        conv63_1_27_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_27_")
        # conv63_1_27_, output shape: {[16,14,14]}

        batchnorm63_1_27_ = mx.symbol.BatchNorm(data=conv63_1_27_,
            fix_gamma=True,
            name="batchnorm63_1_27_")
        relu63_1_27_ = mx.symbol.Activation(data=batchnorm63_1_27_,
            act_type='relu',
            name="relu63_1_27_")

        conv64_1_27_ = mx.symbol.pad(data=relu63_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_27_ = mx.symbol.Convolution(data=conv64_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_27_")
        # conv64_1_27_, output shape: {[16,14,14]}

        batchnorm64_1_27_ = mx.symbol.BatchNorm(data=conv64_1_27_,
            fix_gamma=True,
            name="batchnorm64_1_27_")
        relu64_1_27_ = mx.symbol.Activation(data=batchnorm64_1_27_,
            act_type='relu',
            name="relu64_1_27_")

        conv65_1_27_ = mx.symbol.Convolution(data=relu64_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_27_")
        # conv65_1_27_, output shape: {[1024,14,14]}

        batchnorm65_1_27_ = mx.symbol.BatchNorm(data=conv65_1_27_,
            fix_gamma=True,
            name="batchnorm65_1_27_")
        conv63_1_28_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_28_")
        # conv63_1_28_, output shape: {[16,14,14]}

        batchnorm63_1_28_ = mx.symbol.BatchNorm(data=conv63_1_28_,
            fix_gamma=True,
            name="batchnorm63_1_28_")
        relu63_1_28_ = mx.symbol.Activation(data=batchnorm63_1_28_,
            act_type='relu',
            name="relu63_1_28_")

        conv64_1_28_ = mx.symbol.pad(data=relu63_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_28_ = mx.symbol.Convolution(data=conv64_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_28_")
        # conv64_1_28_, output shape: {[16,14,14]}

        batchnorm64_1_28_ = mx.symbol.BatchNorm(data=conv64_1_28_,
            fix_gamma=True,
            name="batchnorm64_1_28_")
        relu64_1_28_ = mx.symbol.Activation(data=batchnorm64_1_28_,
            act_type='relu',
            name="relu64_1_28_")

        conv65_1_28_ = mx.symbol.Convolution(data=relu64_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_28_")
        # conv65_1_28_, output shape: {[1024,14,14]}

        batchnorm65_1_28_ = mx.symbol.BatchNorm(data=conv65_1_28_,
            fix_gamma=True,
            name="batchnorm65_1_28_")
        conv63_1_29_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_29_")
        # conv63_1_29_, output shape: {[16,14,14]}

        batchnorm63_1_29_ = mx.symbol.BatchNorm(data=conv63_1_29_,
            fix_gamma=True,
            name="batchnorm63_1_29_")
        relu63_1_29_ = mx.symbol.Activation(data=batchnorm63_1_29_,
            act_type='relu',
            name="relu63_1_29_")

        conv64_1_29_ = mx.symbol.pad(data=relu63_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_29_ = mx.symbol.Convolution(data=conv64_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_29_")
        # conv64_1_29_, output shape: {[16,14,14]}

        batchnorm64_1_29_ = mx.symbol.BatchNorm(data=conv64_1_29_,
            fix_gamma=True,
            name="batchnorm64_1_29_")
        relu64_1_29_ = mx.symbol.Activation(data=batchnorm64_1_29_,
            act_type='relu',
            name="relu64_1_29_")

        conv65_1_29_ = mx.symbol.Convolution(data=relu64_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_29_")
        # conv65_1_29_, output shape: {[1024,14,14]}

        batchnorm65_1_29_ = mx.symbol.BatchNorm(data=conv65_1_29_,
            fix_gamma=True,
            name="batchnorm65_1_29_")
        conv63_1_30_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_30_")
        # conv63_1_30_, output shape: {[16,14,14]}

        batchnorm63_1_30_ = mx.symbol.BatchNorm(data=conv63_1_30_,
            fix_gamma=True,
            name="batchnorm63_1_30_")
        relu63_1_30_ = mx.symbol.Activation(data=batchnorm63_1_30_,
            act_type='relu',
            name="relu63_1_30_")

        conv64_1_30_ = mx.symbol.pad(data=relu63_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_30_ = mx.symbol.Convolution(data=conv64_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_30_")
        # conv64_1_30_, output shape: {[16,14,14]}

        batchnorm64_1_30_ = mx.symbol.BatchNorm(data=conv64_1_30_,
            fix_gamma=True,
            name="batchnorm64_1_30_")
        relu64_1_30_ = mx.symbol.Activation(data=batchnorm64_1_30_,
            act_type='relu',
            name="relu64_1_30_")

        conv65_1_30_ = mx.symbol.Convolution(data=relu64_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_30_")
        # conv65_1_30_, output shape: {[1024,14,14]}

        batchnorm65_1_30_ = mx.symbol.BatchNorm(data=conv65_1_30_,
            fix_gamma=True,
            name="batchnorm65_1_30_")
        conv63_1_31_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_31_")
        # conv63_1_31_, output shape: {[16,14,14]}

        batchnorm63_1_31_ = mx.symbol.BatchNorm(data=conv63_1_31_,
            fix_gamma=True,
            name="batchnorm63_1_31_")
        relu63_1_31_ = mx.symbol.Activation(data=batchnorm63_1_31_,
            act_type='relu',
            name="relu63_1_31_")

        conv64_1_31_ = mx.symbol.pad(data=relu63_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_31_ = mx.symbol.Convolution(data=conv64_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_31_")
        # conv64_1_31_, output shape: {[16,14,14]}

        batchnorm64_1_31_ = mx.symbol.BatchNorm(data=conv64_1_31_,
            fix_gamma=True,
            name="batchnorm64_1_31_")
        relu64_1_31_ = mx.symbol.Activation(data=batchnorm64_1_31_,
            act_type='relu',
            name="relu64_1_31_")

        conv65_1_31_ = mx.symbol.Convolution(data=relu64_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_31_")
        # conv65_1_31_, output shape: {[1024,14,14]}

        batchnorm65_1_31_ = mx.symbol.BatchNorm(data=conv65_1_31_,
            fix_gamma=True,
            name="batchnorm65_1_31_")
        conv63_1_32_ = mx.symbol.Convolution(data=relu61_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv63_1_32_")
        # conv63_1_32_, output shape: {[16,14,14]}

        batchnorm63_1_32_ = mx.symbol.BatchNorm(data=conv63_1_32_,
            fix_gamma=True,
            name="batchnorm63_1_32_")
        relu63_1_32_ = mx.symbol.Activation(data=batchnorm63_1_32_,
            act_type='relu',
            name="relu63_1_32_")

        conv64_1_32_ = mx.symbol.pad(data=relu63_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv64_1_32_ = mx.symbol.Convolution(data=conv64_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv64_1_32_")
        # conv64_1_32_, output shape: {[16,14,14]}

        batchnorm64_1_32_ = mx.symbol.BatchNorm(data=conv64_1_32_,
            fix_gamma=True,
            name="batchnorm64_1_32_")
        relu64_1_32_ = mx.symbol.Activation(data=batchnorm64_1_32_,
            act_type='relu',
            name="relu64_1_32_")

        conv65_1_32_ = mx.symbol.Convolution(data=relu64_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv65_1_32_")
        # conv65_1_32_, output shape: {[1024,14,14]}

        batchnorm65_1_32_ = mx.symbol.BatchNorm(data=conv65_1_32_,
            fix_gamma=True,
            name="batchnorm65_1_32_")
        add66_1_ = batchnorm65_1_1_ + batchnorm65_1_2_ + batchnorm65_1_3_ + batchnorm65_1_4_ + batchnorm65_1_5_ + batchnorm65_1_6_ + batchnorm65_1_7_ + batchnorm65_1_8_ + batchnorm65_1_9_ + batchnorm65_1_10_ + batchnorm65_1_11_ + batchnorm65_1_12_ + batchnorm65_1_13_ + batchnorm65_1_14_ + batchnorm65_1_15_ + batchnorm65_1_16_ + batchnorm65_1_17_ + batchnorm65_1_18_ + batchnorm65_1_19_ + batchnorm65_1_20_ + batchnorm65_1_21_ + batchnorm65_1_22_ + batchnorm65_1_23_ + batchnorm65_1_24_ + batchnorm65_1_25_ + batchnorm65_1_26_ + batchnorm65_1_27_ + batchnorm65_1_28_ + batchnorm65_1_29_ + batchnorm65_1_30_ + batchnorm65_1_31_ + batchnorm65_1_32_
        # add66_1_, output shape: {[1024,14,14]}

        add67_ = add66_1_ + relu61_
        # add67_, output shape: {[1024,14,14]}

        relu67_ = mx.symbol.Activation(data=add67_,
            act_type='relu',
            name="relu67_")

        conv69_1_1_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_1_")
        # conv69_1_1_, output shape: {[16,14,14]}

        batchnorm69_1_1_ = mx.symbol.BatchNorm(data=conv69_1_1_,
            fix_gamma=True,
            name="batchnorm69_1_1_")
        relu69_1_1_ = mx.symbol.Activation(data=batchnorm69_1_1_,
            act_type='relu',
            name="relu69_1_1_")

        conv70_1_1_ = mx.symbol.pad(data=relu69_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_1_ = mx.symbol.Convolution(data=conv70_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_1_")
        # conv70_1_1_, output shape: {[16,14,14]}

        batchnorm70_1_1_ = mx.symbol.BatchNorm(data=conv70_1_1_,
            fix_gamma=True,
            name="batchnorm70_1_1_")
        relu70_1_1_ = mx.symbol.Activation(data=batchnorm70_1_1_,
            act_type='relu',
            name="relu70_1_1_")

        conv71_1_1_ = mx.symbol.Convolution(data=relu70_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_1_")
        # conv71_1_1_, output shape: {[1024,14,14]}

        batchnorm71_1_1_ = mx.symbol.BatchNorm(data=conv71_1_1_,
            fix_gamma=True,
            name="batchnorm71_1_1_")
        conv69_1_2_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_2_")
        # conv69_1_2_, output shape: {[16,14,14]}

        batchnorm69_1_2_ = mx.symbol.BatchNorm(data=conv69_1_2_,
            fix_gamma=True,
            name="batchnorm69_1_2_")
        relu69_1_2_ = mx.symbol.Activation(data=batchnorm69_1_2_,
            act_type='relu',
            name="relu69_1_2_")

        conv70_1_2_ = mx.symbol.pad(data=relu69_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_2_ = mx.symbol.Convolution(data=conv70_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_2_")
        # conv70_1_2_, output shape: {[16,14,14]}

        batchnorm70_1_2_ = mx.symbol.BatchNorm(data=conv70_1_2_,
            fix_gamma=True,
            name="batchnorm70_1_2_")
        relu70_1_2_ = mx.symbol.Activation(data=batchnorm70_1_2_,
            act_type='relu',
            name="relu70_1_2_")

        conv71_1_2_ = mx.symbol.Convolution(data=relu70_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_2_")
        # conv71_1_2_, output shape: {[1024,14,14]}

        batchnorm71_1_2_ = mx.symbol.BatchNorm(data=conv71_1_2_,
            fix_gamma=True,
            name="batchnorm71_1_2_")
        conv69_1_3_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_3_")
        # conv69_1_3_, output shape: {[16,14,14]}

        batchnorm69_1_3_ = mx.symbol.BatchNorm(data=conv69_1_3_,
            fix_gamma=True,
            name="batchnorm69_1_3_")
        relu69_1_3_ = mx.symbol.Activation(data=batchnorm69_1_3_,
            act_type='relu',
            name="relu69_1_3_")

        conv70_1_3_ = mx.symbol.pad(data=relu69_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_3_ = mx.symbol.Convolution(data=conv70_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_3_")
        # conv70_1_3_, output shape: {[16,14,14]}

        batchnorm70_1_3_ = mx.symbol.BatchNorm(data=conv70_1_3_,
            fix_gamma=True,
            name="batchnorm70_1_3_")
        relu70_1_3_ = mx.symbol.Activation(data=batchnorm70_1_3_,
            act_type='relu',
            name="relu70_1_3_")

        conv71_1_3_ = mx.symbol.Convolution(data=relu70_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_3_")
        # conv71_1_3_, output shape: {[1024,14,14]}

        batchnorm71_1_3_ = mx.symbol.BatchNorm(data=conv71_1_3_,
            fix_gamma=True,
            name="batchnorm71_1_3_")
        conv69_1_4_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_4_")
        # conv69_1_4_, output shape: {[16,14,14]}

        batchnorm69_1_4_ = mx.symbol.BatchNorm(data=conv69_1_4_,
            fix_gamma=True,
            name="batchnorm69_1_4_")
        relu69_1_4_ = mx.symbol.Activation(data=batchnorm69_1_4_,
            act_type='relu',
            name="relu69_1_4_")

        conv70_1_4_ = mx.symbol.pad(data=relu69_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_4_ = mx.symbol.Convolution(data=conv70_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_4_")
        # conv70_1_4_, output shape: {[16,14,14]}

        batchnorm70_1_4_ = mx.symbol.BatchNorm(data=conv70_1_4_,
            fix_gamma=True,
            name="batchnorm70_1_4_")
        relu70_1_4_ = mx.symbol.Activation(data=batchnorm70_1_4_,
            act_type='relu',
            name="relu70_1_4_")

        conv71_1_4_ = mx.symbol.Convolution(data=relu70_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_4_")
        # conv71_1_4_, output shape: {[1024,14,14]}

        batchnorm71_1_4_ = mx.symbol.BatchNorm(data=conv71_1_4_,
            fix_gamma=True,
            name="batchnorm71_1_4_")
        conv69_1_5_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_5_")
        # conv69_1_5_, output shape: {[16,14,14]}

        batchnorm69_1_5_ = mx.symbol.BatchNorm(data=conv69_1_5_,
            fix_gamma=True,
            name="batchnorm69_1_5_")
        relu69_1_5_ = mx.symbol.Activation(data=batchnorm69_1_5_,
            act_type='relu',
            name="relu69_1_5_")

        conv70_1_5_ = mx.symbol.pad(data=relu69_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_5_ = mx.symbol.Convolution(data=conv70_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_5_")
        # conv70_1_5_, output shape: {[16,14,14]}

        batchnorm70_1_5_ = mx.symbol.BatchNorm(data=conv70_1_5_,
            fix_gamma=True,
            name="batchnorm70_1_5_")
        relu70_1_5_ = mx.symbol.Activation(data=batchnorm70_1_5_,
            act_type='relu',
            name="relu70_1_5_")

        conv71_1_5_ = mx.symbol.Convolution(data=relu70_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_5_")
        # conv71_1_5_, output shape: {[1024,14,14]}

        batchnorm71_1_5_ = mx.symbol.BatchNorm(data=conv71_1_5_,
            fix_gamma=True,
            name="batchnorm71_1_5_")
        conv69_1_6_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_6_")
        # conv69_1_6_, output shape: {[16,14,14]}

        batchnorm69_1_6_ = mx.symbol.BatchNorm(data=conv69_1_6_,
            fix_gamma=True,
            name="batchnorm69_1_6_")
        relu69_1_6_ = mx.symbol.Activation(data=batchnorm69_1_6_,
            act_type='relu',
            name="relu69_1_6_")

        conv70_1_6_ = mx.symbol.pad(data=relu69_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_6_ = mx.symbol.Convolution(data=conv70_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_6_")
        # conv70_1_6_, output shape: {[16,14,14]}

        batchnorm70_1_6_ = mx.symbol.BatchNorm(data=conv70_1_6_,
            fix_gamma=True,
            name="batchnorm70_1_6_")
        relu70_1_6_ = mx.symbol.Activation(data=batchnorm70_1_6_,
            act_type='relu',
            name="relu70_1_6_")

        conv71_1_6_ = mx.symbol.Convolution(data=relu70_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_6_")
        # conv71_1_6_, output shape: {[1024,14,14]}

        batchnorm71_1_6_ = mx.symbol.BatchNorm(data=conv71_1_6_,
            fix_gamma=True,
            name="batchnorm71_1_6_")
        conv69_1_7_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_7_")
        # conv69_1_7_, output shape: {[16,14,14]}

        batchnorm69_1_7_ = mx.symbol.BatchNorm(data=conv69_1_7_,
            fix_gamma=True,
            name="batchnorm69_1_7_")
        relu69_1_7_ = mx.symbol.Activation(data=batchnorm69_1_7_,
            act_type='relu',
            name="relu69_1_7_")

        conv70_1_7_ = mx.symbol.pad(data=relu69_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_7_ = mx.symbol.Convolution(data=conv70_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_7_")
        # conv70_1_7_, output shape: {[16,14,14]}

        batchnorm70_1_7_ = mx.symbol.BatchNorm(data=conv70_1_7_,
            fix_gamma=True,
            name="batchnorm70_1_7_")
        relu70_1_7_ = mx.symbol.Activation(data=batchnorm70_1_7_,
            act_type='relu',
            name="relu70_1_7_")

        conv71_1_7_ = mx.symbol.Convolution(data=relu70_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_7_")
        # conv71_1_7_, output shape: {[1024,14,14]}

        batchnorm71_1_7_ = mx.symbol.BatchNorm(data=conv71_1_7_,
            fix_gamma=True,
            name="batchnorm71_1_7_")
        conv69_1_8_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_8_")
        # conv69_1_8_, output shape: {[16,14,14]}

        batchnorm69_1_8_ = mx.symbol.BatchNorm(data=conv69_1_8_,
            fix_gamma=True,
            name="batchnorm69_1_8_")
        relu69_1_8_ = mx.symbol.Activation(data=batchnorm69_1_8_,
            act_type='relu',
            name="relu69_1_8_")

        conv70_1_8_ = mx.symbol.pad(data=relu69_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_8_ = mx.symbol.Convolution(data=conv70_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_8_")
        # conv70_1_8_, output shape: {[16,14,14]}

        batchnorm70_1_8_ = mx.symbol.BatchNorm(data=conv70_1_8_,
            fix_gamma=True,
            name="batchnorm70_1_8_")
        relu70_1_8_ = mx.symbol.Activation(data=batchnorm70_1_8_,
            act_type='relu',
            name="relu70_1_8_")

        conv71_1_8_ = mx.symbol.Convolution(data=relu70_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_8_")
        # conv71_1_8_, output shape: {[1024,14,14]}

        batchnorm71_1_8_ = mx.symbol.BatchNorm(data=conv71_1_8_,
            fix_gamma=True,
            name="batchnorm71_1_8_")
        conv69_1_9_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_9_")
        # conv69_1_9_, output shape: {[16,14,14]}

        batchnorm69_1_9_ = mx.symbol.BatchNorm(data=conv69_1_9_,
            fix_gamma=True,
            name="batchnorm69_1_9_")
        relu69_1_9_ = mx.symbol.Activation(data=batchnorm69_1_9_,
            act_type='relu',
            name="relu69_1_9_")

        conv70_1_9_ = mx.symbol.pad(data=relu69_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_9_ = mx.symbol.Convolution(data=conv70_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_9_")
        # conv70_1_9_, output shape: {[16,14,14]}

        batchnorm70_1_9_ = mx.symbol.BatchNorm(data=conv70_1_9_,
            fix_gamma=True,
            name="batchnorm70_1_9_")
        relu70_1_9_ = mx.symbol.Activation(data=batchnorm70_1_9_,
            act_type='relu',
            name="relu70_1_9_")

        conv71_1_9_ = mx.symbol.Convolution(data=relu70_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_9_")
        # conv71_1_9_, output shape: {[1024,14,14]}

        batchnorm71_1_9_ = mx.symbol.BatchNorm(data=conv71_1_9_,
            fix_gamma=True,
            name="batchnorm71_1_9_")
        conv69_1_10_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_10_")
        # conv69_1_10_, output shape: {[16,14,14]}

        batchnorm69_1_10_ = mx.symbol.BatchNorm(data=conv69_1_10_,
            fix_gamma=True,
            name="batchnorm69_1_10_")
        relu69_1_10_ = mx.symbol.Activation(data=batchnorm69_1_10_,
            act_type='relu',
            name="relu69_1_10_")

        conv70_1_10_ = mx.symbol.pad(data=relu69_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_10_ = mx.symbol.Convolution(data=conv70_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_10_")
        # conv70_1_10_, output shape: {[16,14,14]}

        batchnorm70_1_10_ = mx.symbol.BatchNorm(data=conv70_1_10_,
            fix_gamma=True,
            name="batchnorm70_1_10_")
        relu70_1_10_ = mx.symbol.Activation(data=batchnorm70_1_10_,
            act_type='relu',
            name="relu70_1_10_")

        conv71_1_10_ = mx.symbol.Convolution(data=relu70_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_10_")
        # conv71_1_10_, output shape: {[1024,14,14]}

        batchnorm71_1_10_ = mx.symbol.BatchNorm(data=conv71_1_10_,
            fix_gamma=True,
            name="batchnorm71_1_10_")
        conv69_1_11_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_11_")
        # conv69_1_11_, output shape: {[16,14,14]}

        batchnorm69_1_11_ = mx.symbol.BatchNorm(data=conv69_1_11_,
            fix_gamma=True,
            name="batchnorm69_1_11_")
        relu69_1_11_ = mx.symbol.Activation(data=batchnorm69_1_11_,
            act_type='relu',
            name="relu69_1_11_")

        conv70_1_11_ = mx.symbol.pad(data=relu69_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_11_ = mx.symbol.Convolution(data=conv70_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_11_")
        # conv70_1_11_, output shape: {[16,14,14]}

        batchnorm70_1_11_ = mx.symbol.BatchNorm(data=conv70_1_11_,
            fix_gamma=True,
            name="batchnorm70_1_11_")
        relu70_1_11_ = mx.symbol.Activation(data=batchnorm70_1_11_,
            act_type='relu',
            name="relu70_1_11_")

        conv71_1_11_ = mx.symbol.Convolution(data=relu70_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_11_")
        # conv71_1_11_, output shape: {[1024,14,14]}

        batchnorm71_1_11_ = mx.symbol.BatchNorm(data=conv71_1_11_,
            fix_gamma=True,
            name="batchnorm71_1_11_")
        conv69_1_12_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_12_")
        # conv69_1_12_, output shape: {[16,14,14]}

        batchnorm69_1_12_ = mx.symbol.BatchNorm(data=conv69_1_12_,
            fix_gamma=True,
            name="batchnorm69_1_12_")
        relu69_1_12_ = mx.symbol.Activation(data=batchnorm69_1_12_,
            act_type='relu',
            name="relu69_1_12_")

        conv70_1_12_ = mx.symbol.pad(data=relu69_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_12_ = mx.symbol.Convolution(data=conv70_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_12_")
        # conv70_1_12_, output shape: {[16,14,14]}

        batchnorm70_1_12_ = mx.symbol.BatchNorm(data=conv70_1_12_,
            fix_gamma=True,
            name="batchnorm70_1_12_")
        relu70_1_12_ = mx.symbol.Activation(data=batchnorm70_1_12_,
            act_type='relu',
            name="relu70_1_12_")

        conv71_1_12_ = mx.symbol.Convolution(data=relu70_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_12_")
        # conv71_1_12_, output shape: {[1024,14,14]}

        batchnorm71_1_12_ = mx.symbol.BatchNorm(data=conv71_1_12_,
            fix_gamma=True,
            name="batchnorm71_1_12_")
        conv69_1_13_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_13_")
        # conv69_1_13_, output shape: {[16,14,14]}

        batchnorm69_1_13_ = mx.symbol.BatchNorm(data=conv69_1_13_,
            fix_gamma=True,
            name="batchnorm69_1_13_")
        relu69_1_13_ = mx.symbol.Activation(data=batchnorm69_1_13_,
            act_type='relu',
            name="relu69_1_13_")

        conv70_1_13_ = mx.symbol.pad(data=relu69_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_13_ = mx.symbol.Convolution(data=conv70_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_13_")
        # conv70_1_13_, output shape: {[16,14,14]}

        batchnorm70_1_13_ = mx.symbol.BatchNorm(data=conv70_1_13_,
            fix_gamma=True,
            name="batchnorm70_1_13_")
        relu70_1_13_ = mx.symbol.Activation(data=batchnorm70_1_13_,
            act_type='relu',
            name="relu70_1_13_")

        conv71_1_13_ = mx.symbol.Convolution(data=relu70_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_13_")
        # conv71_1_13_, output shape: {[1024,14,14]}

        batchnorm71_1_13_ = mx.symbol.BatchNorm(data=conv71_1_13_,
            fix_gamma=True,
            name="batchnorm71_1_13_")
        conv69_1_14_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_14_")
        # conv69_1_14_, output shape: {[16,14,14]}

        batchnorm69_1_14_ = mx.symbol.BatchNorm(data=conv69_1_14_,
            fix_gamma=True,
            name="batchnorm69_1_14_")
        relu69_1_14_ = mx.symbol.Activation(data=batchnorm69_1_14_,
            act_type='relu',
            name="relu69_1_14_")

        conv70_1_14_ = mx.symbol.pad(data=relu69_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_14_ = mx.symbol.Convolution(data=conv70_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_14_")
        # conv70_1_14_, output shape: {[16,14,14]}

        batchnorm70_1_14_ = mx.symbol.BatchNorm(data=conv70_1_14_,
            fix_gamma=True,
            name="batchnorm70_1_14_")
        relu70_1_14_ = mx.symbol.Activation(data=batchnorm70_1_14_,
            act_type='relu',
            name="relu70_1_14_")

        conv71_1_14_ = mx.symbol.Convolution(data=relu70_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_14_")
        # conv71_1_14_, output shape: {[1024,14,14]}

        batchnorm71_1_14_ = mx.symbol.BatchNorm(data=conv71_1_14_,
            fix_gamma=True,
            name="batchnorm71_1_14_")
        conv69_1_15_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_15_")
        # conv69_1_15_, output shape: {[16,14,14]}

        batchnorm69_1_15_ = mx.symbol.BatchNorm(data=conv69_1_15_,
            fix_gamma=True,
            name="batchnorm69_1_15_")
        relu69_1_15_ = mx.symbol.Activation(data=batchnorm69_1_15_,
            act_type='relu',
            name="relu69_1_15_")

        conv70_1_15_ = mx.symbol.pad(data=relu69_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_15_ = mx.symbol.Convolution(data=conv70_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_15_")
        # conv70_1_15_, output shape: {[16,14,14]}

        batchnorm70_1_15_ = mx.symbol.BatchNorm(data=conv70_1_15_,
            fix_gamma=True,
            name="batchnorm70_1_15_")
        relu70_1_15_ = mx.symbol.Activation(data=batchnorm70_1_15_,
            act_type='relu',
            name="relu70_1_15_")

        conv71_1_15_ = mx.symbol.Convolution(data=relu70_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_15_")
        # conv71_1_15_, output shape: {[1024,14,14]}

        batchnorm71_1_15_ = mx.symbol.BatchNorm(data=conv71_1_15_,
            fix_gamma=True,
            name="batchnorm71_1_15_")
        conv69_1_16_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_16_")
        # conv69_1_16_, output shape: {[16,14,14]}

        batchnorm69_1_16_ = mx.symbol.BatchNorm(data=conv69_1_16_,
            fix_gamma=True,
            name="batchnorm69_1_16_")
        relu69_1_16_ = mx.symbol.Activation(data=batchnorm69_1_16_,
            act_type='relu',
            name="relu69_1_16_")

        conv70_1_16_ = mx.symbol.pad(data=relu69_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_16_ = mx.symbol.Convolution(data=conv70_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_16_")
        # conv70_1_16_, output shape: {[16,14,14]}

        batchnorm70_1_16_ = mx.symbol.BatchNorm(data=conv70_1_16_,
            fix_gamma=True,
            name="batchnorm70_1_16_")
        relu70_1_16_ = mx.symbol.Activation(data=batchnorm70_1_16_,
            act_type='relu',
            name="relu70_1_16_")

        conv71_1_16_ = mx.symbol.Convolution(data=relu70_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_16_")
        # conv71_1_16_, output shape: {[1024,14,14]}

        batchnorm71_1_16_ = mx.symbol.BatchNorm(data=conv71_1_16_,
            fix_gamma=True,
            name="batchnorm71_1_16_")
        conv69_1_17_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_17_")
        # conv69_1_17_, output shape: {[16,14,14]}

        batchnorm69_1_17_ = mx.symbol.BatchNorm(data=conv69_1_17_,
            fix_gamma=True,
            name="batchnorm69_1_17_")
        relu69_1_17_ = mx.symbol.Activation(data=batchnorm69_1_17_,
            act_type='relu',
            name="relu69_1_17_")

        conv70_1_17_ = mx.symbol.pad(data=relu69_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_17_ = mx.symbol.Convolution(data=conv70_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_17_")
        # conv70_1_17_, output shape: {[16,14,14]}

        batchnorm70_1_17_ = mx.symbol.BatchNorm(data=conv70_1_17_,
            fix_gamma=True,
            name="batchnorm70_1_17_")
        relu70_1_17_ = mx.symbol.Activation(data=batchnorm70_1_17_,
            act_type='relu',
            name="relu70_1_17_")

        conv71_1_17_ = mx.symbol.Convolution(data=relu70_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_17_")
        # conv71_1_17_, output shape: {[1024,14,14]}

        batchnorm71_1_17_ = mx.symbol.BatchNorm(data=conv71_1_17_,
            fix_gamma=True,
            name="batchnorm71_1_17_")
        conv69_1_18_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_18_")
        # conv69_1_18_, output shape: {[16,14,14]}

        batchnorm69_1_18_ = mx.symbol.BatchNorm(data=conv69_1_18_,
            fix_gamma=True,
            name="batchnorm69_1_18_")
        relu69_1_18_ = mx.symbol.Activation(data=batchnorm69_1_18_,
            act_type='relu',
            name="relu69_1_18_")

        conv70_1_18_ = mx.symbol.pad(data=relu69_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_18_ = mx.symbol.Convolution(data=conv70_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_18_")
        # conv70_1_18_, output shape: {[16,14,14]}

        batchnorm70_1_18_ = mx.symbol.BatchNorm(data=conv70_1_18_,
            fix_gamma=True,
            name="batchnorm70_1_18_")
        relu70_1_18_ = mx.symbol.Activation(data=batchnorm70_1_18_,
            act_type='relu',
            name="relu70_1_18_")

        conv71_1_18_ = mx.symbol.Convolution(data=relu70_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_18_")
        # conv71_1_18_, output shape: {[1024,14,14]}

        batchnorm71_1_18_ = mx.symbol.BatchNorm(data=conv71_1_18_,
            fix_gamma=True,
            name="batchnorm71_1_18_")
        conv69_1_19_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_19_")
        # conv69_1_19_, output shape: {[16,14,14]}

        batchnorm69_1_19_ = mx.symbol.BatchNorm(data=conv69_1_19_,
            fix_gamma=True,
            name="batchnorm69_1_19_")
        relu69_1_19_ = mx.symbol.Activation(data=batchnorm69_1_19_,
            act_type='relu',
            name="relu69_1_19_")

        conv70_1_19_ = mx.symbol.pad(data=relu69_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_19_ = mx.symbol.Convolution(data=conv70_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_19_")
        # conv70_1_19_, output shape: {[16,14,14]}

        batchnorm70_1_19_ = mx.symbol.BatchNorm(data=conv70_1_19_,
            fix_gamma=True,
            name="batchnorm70_1_19_")
        relu70_1_19_ = mx.symbol.Activation(data=batchnorm70_1_19_,
            act_type='relu',
            name="relu70_1_19_")

        conv71_1_19_ = mx.symbol.Convolution(data=relu70_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_19_")
        # conv71_1_19_, output shape: {[1024,14,14]}

        batchnorm71_1_19_ = mx.symbol.BatchNorm(data=conv71_1_19_,
            fix_gamma=True,
            name="batchnorm71_1_19_")
        conv69_1_20_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_20_")
        # conv69_1_20_, output shape: {[16,14,14]}

        batchnorm69_1_20_ = mx.symbol.BatchNorm(data=conv69_1_20_,
            fix_gamma=True,
            name="batchnorm69_1_20_")
        relu69_1_20_ = mx.symbol.Activation(data=batchnorm69_1_20_,
            act_type='relu',
            name="relu69_1_20_")

        conv70_1_20_ = mx.symbol.pad(data=relu69_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_20_ = mx.symbol.Convolution(data=conv70_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_20_")
        # conv70_1_20_, output shape: {[16,14,14]}

        batchnorm70_1_20_ = mx.symbol.BatchNorm(data=conv70_1_20_,
            fix_gamma=True,
            name="batchnorm70_1_20_")
        relu70_1_20_ = mx.symbol.Activation(data=batchnorm70_1_20_,
            act_type='relu',
            name="relu70_1_20_")

        conv71_1_20_ = mx.symbol.Convolution(data=relu70_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_20_")
        # conv71_1_20_, output shape: {[1024,14,14]}

        batchnorm71_1_20_ = mx.symbol.BatchNorm(data=conv71_1_20_,
            fix_gamma=True,
            name="batchnorm71_1_20_")
        conv69_1_21_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_21_")
        # conv69_1_21_, output shape: {[16,14,14]}

        batchnorm69_1_21_ = mx.symbol.BatchNorm(data=conv69_1_21_,
            fix_gamma=True,
            name="batchnorm69_1_21_")
        relu69_1_21_ = mx.symbol.Activation(data=batchnorm69_1_21_,
            act_type='relu',
            name="relu69_1_21_")

        conv70_1_21_ = mx.symbol.pad(data=relu69_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_21_ = mx.symbol.Convolution(data=conv70_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_21_")
        # conv70_1_21_, output shape: {[16,14,14]}

        batchnorm70_1_21_ = mx.symbol.BatchNorm(data=conv70_1_21_,
            fix_gamma=True,
            name="batchnorm70_1_21_")
        relu70_1_21_ = mx.symbol.Activation(data=batchnorm70_1_21_,
            act_type='relu',
            name="relu70_1_21_")

        conv71_1_21_ = mx.symbol.Convolution(data=relu70_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_21_")
        # conv71_1_21_, output shape: {[1024,14,14]}

        batchnorm71_1_21_ = mx.symbol.BatchNorm(data=conv71_1_21_,
            fix_gamma=True,
            name="batchnorm71_1_21_")
        conv69_1_22_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_22_")
        # conv69_1_22_, output shape: {[16,14,14]}

        batchnorm69_1_22_ = mx.symbol.BatchNorm(data=conv69_1_22_,
            fix_gamma=True,
            name="batchnorm69_1_22_")
        relu69_1_22_ = mx.symbol.Activation(data=batchnorm69_1_22_,
            act_type='relu',
            name="relu69_1_22_")

        conv70_1_22_ = mx.symbol.pad(data=relu69_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_22_ = mx.symbol.Convolution(data=conv70_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_22_")
        # conv70_1_22_, output shape: {[16,14,14]}

        batchnorm70_1_22_ = mx.symbol.BatchNorm(data=conv70_1_22_,
            fix_gamma=True,
            name="batchnorm70_1_22_")
        relu70_1_22_ = mx.symbol.Activation(data=batchnorm70_1_22_,
            act_type='relu',
            name="relu70_1_22_")

        conv71_1_22_ = mx.symbol.Convolution(data=relu70_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_22_")
        # conv71_1_22_, output shape: {[1024,14,14]}

        batchnorm71_1_22_ = mx.symbol.BatchNorm(data=conv71_1_22_,
            fix_gamma=True,
            name="batchnorm71_1_22_")
        conv69_1_23_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_23_")
        # conv69_1_23_, output shape: {[16,14,14]}

        batchnorm69_1_23_ = mx.symbol.BatchNorm(data=conv69_1_23_,
            fix_gamma=True,
            name="batchnorm69_1_23_")
        relu69_1_23_ = mx.symbol.Activation(data=batchnorm69_1_23_,
            act_type='relu',
            name="relu69_1_23_")

        conv70_1_23_ = mx.symbol.pad(data=relu69_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_23_ = mx.symbol.Convolution(data=conv70_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_23_")
        # conv70_1_23_, output shape: {[16,14,14]}

        batchnorm70_1_23_ = mx.symbol.BatchNorm(data=conv70_1_23_,
            fix_gamma=True,
            name="batchnorm70_1_23_")
        relu70_1_23_ = mx.symbol.Activation(data=batchnorm70_1_23_,
            act_type='relu',
            name="relu70_1_23_")

        conv71_1_23_ = mx.symbol.Convolution(data=relu70_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_23_")
        # conv71_1_23_, output shape: {[1024,14,14]}

        batchnorm71_1_23_ = mx.symbol.BatchNorm(data=conv71_1_23_,
            fix_gamma=True,
            name="batchnorm71_1_23_")
        conv69_1_24_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_24_")
        # conv69_1_24_, output shape: {[16,14,14]}

        batchnorm69_1_24_ = mx.symbol.BatchNorm(data=conv69_1_24_,
            fix_gamma=True,
            name="batchnorm69_1_24_")
        relu69_1_24_ = mx.symbol.Activation(data=batchnorm69_1_24_,
            act_type='relu',
            name="relu69_1_24_")

        conv70_1_24_ = mx.symbol.pad(data=relu69_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_24_ = mx.symbol.Convolution(data=conv70_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_24_")
        # conv70_1_24_, output shape: {[16,14,14]}

        batchnorm70_1_24_ = mx.symbol.BatchNorm(data=conv70_1_24_,
            fix_gamma=True,
            name="batchnorm70_1_24_")
        relu70_1_24_ = mx.symbol.Activation(data=batchnorm70_1_24_,
            act_type='relu',
            name="relu70_1_24_")

        conv71_1_24_ = mx.symbol.Convolution(data=relu70_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_24_")
        # conv71_1_24_, output shape: {[1024,14,14]}

        batchnorm71_1_24_ = mx.symbol.BatchNorm(data=conv71_1_24_,
            fix_gamma=True,
            name="batchnorm71_1_24_")
        conv69_1_25_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_25_")
        # conv69_1_25_, output shape: {[16,14,14]}

        batchnorm69_1_25_ = mx.symbol.BatchNorm(data=conv69_1_25_,
            fix_gamma=True,
            name="batchnorm69_1_25_")
        relu69_1_25_ = mx.symbol.Activation(data=batchnorm69_1_25_,
            act_type='relu',
            name="relu69_1_25_")

        conv70_1_25_ = mx.symbol.pad(data=relu69_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_25_ = mx.symbol.Convolution(data=conv70_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_25_")
        # conv70_1_25_, output shape: {[16,14,14]}

        batchnorm70_1_25_ = mx.symbol.BatchNorm(data=conv70_1_25_,
            fix_gamma=True,
            name="batchnorm70_1_25_")
        relu70_1_25_ = mx.symbol.Activation(data=batchnorm70_1_25_,
            act_type='relu',
            name="relu70_1_25_")

        conv71_1_25_ = mx.symbol.Convolution(data=relu70_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_25_")
        # conv71_1_25_, output shape: {[1024,14,14]}

        batchnorm71_1_25_ = mx.symbol.BatchNorm(data=conv71_1_25_,
            fix_gamma=True,
            name="batchnorm71_1_25_")
        conv69_1_26_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_26_")
        # conv69_1_26_, output shape: {[16,14,14]}

        batchnorm69_1_26_ = mx.symbol.BatchNorm(data=conv69_1_26_,
            fix_gamma=True,
            name="batchnorm69_1_26_")
        relu69_1_26_ = mx.symbol.Activation(data=batchnorm69_1_26_,
            act_type='relu',
            name="relu69_1_26_")

        conv70_1_26_ = mx.symbol.pad(data=relu69_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_26_ = mx.symbol.Convolution(data=conv70_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_26_")
        # conv70_1_26_, output shape: {[16,14,14]}

        batchnorm70_1_26_ = mx.symbol.BatchNorm(data=conv70_1_26_,
            fix_gamma=True,
            name="batchnorm70_1_26_")
        relu70_1_26_ = mx.symbol.Activation(data=batchnorm70_1_26_,
            act_type='relu',
            name="relu70_1_26_")

        conv71_1_26_ = mx.symbol.Convolution(data=relu70_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_26_")
        # conv71_1_26_, output shape: {[1024,14,14]}

        batchnorm71_1_26_ = mx.symbol.BatchNorm(data=conv71_1_26_,
            fix_gamma=True,
            name="batchnorm71_1_26_")
        conv69_1_27_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_27_")
        # conv69_1_27_, output shape: {[16,14,14]}

        batchnorm69_1_27_ = mx.symbol.BatchNorm(data=conv69_1_27_,
            fix_gamma=True,
            name="batchnorm69_1_27_")
        relu69_1_27_ = mx.symbol.Activation(data=batchnorm69_1_27_,
            act_type='relu',
            name="relu69_1_27_")

        conv70_1_27_ = mx.symbol.pad(data=relu69_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_27_ = mx.symbol.Convolution(data=conv70_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_27_")
        # conv70_1_27_, output shape: {[16,14,14]}

        batchnorm70_1_27_ = mx.symbol.BatchNorm(data=conv70_1_27_,
            fix_gamma=True,
            name="batchnorm70_1_27_")
        relu70_1_27_ = mx.symbol.Activation(data=batchnorm70_1_27_,
            act_type='relu',
            name="relu70_1_27_")

        conv71_1_27_ = mx.symbol.Convolution(data=relu70_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_27_")
        # conv71_1_27_, output shape: {[1024,14,14]}

        batchnorm71_1_27_ = mx.symbol.BatchNorm(data=conv71_1_27_,
            fix_gamma=True,
            name="batchnorm71_1_27_")
        conv69_1_28_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_28_")
        # conv69_1_28_, output shape: {[16,14,14]}

        batchnorm69_1_28_ = mx.symbol.BatchNorm(data=conv69_1_28_,
            fix_gamma=True,
            name="batchnorm69_1_28_")
        relu69_1_28_ = mx.symbol.Activation(data=batchnorm69_1_28_,
            act_type='relu',
            name="relu69_1_28_")

        conv70_1_28_ = mx.symbol.pad(data=relu69_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_28_ = mx.symbol.Convolution(data=conv70_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_28_")
        # conv70_1_28_, output shape: {[16,14,14]}

        batchnorm70_1_28_ = mx.symbol.BatchNorm(data=conv70_1_28_,
            fix_gamma=True,
            name="batchnorm70_1_28_")
        relu70_1_28_ = mx.symbol.Activation(data=batchnorm70_1_28_,
            act_type='relu',
            name="relu70_1_28_")

        conv71_1_28_ = mx.symbol.Convolution(data=relu70_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_28_")
        # conv71_1_28_, output shape: {[1024,14,14]}

        batchnorm71_1_28_ = mx.symbol.BatchNorm(data=conv71_1_28_,
            fix_gamma=True,
            name="batchnorm71_1_28_")
        conv69_1_29_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_29_")
        # conv69_1_29_, output shape: {[16,14,14]}

        batchnorm69_1_29_ = mx.symbol.BatchNorm(data=conv69_1_29_,
            fix_gamma=True,
            name="batchnorm69_1_29_")
        relu69_1_29_ = mx.symbol.Activation(data=batchnorm69_1_29_,
            act_type='relu',
            name="relu69_1_29_")

        conv70_1_29_ = mx.symbol.pad(data=relu69_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_29_ = mx.symbol.Convolution(data=conv70_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_29_")
        # conv70_1_29_, output shape: {[16,14,14]}

        batchnorm70_1_29_ = mx.symbol.BatchNorm(data=conv70_1_29_,
            fix_gamma=True,
            name="batchnorm70_1_29_")
        relu70_1_29_ = mx.symbol.Activation(data=batchnorm70_1_29_,
            act_type='relu',
            name="relu70_1_29_")

        conv71_1_29_ = mx.symbol.Convolution(data=relu70_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_29_")
        # conv71_1_29_, output shape: {[1024,14,14]}

        batchnorm71_1_29_ = mx.symbol.BatchNorm(data=conv71_1_29_,
            fix_gamma=True,
            name="batchnorm71_1_29_")
        conv69_1_30_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_30_")
        # conv69_1_30_, output shape: {[16,14,14]}

        batchnorm69_1_30_ = mx.symbol.BatchNorm(data=conv69_1_30_,
            fix_gamma=True,
            name="batchnorm69_1_30_")
        relu69_1_30_ = mx.symbol.Activation(data=batchnorm69_1_30_,
            act_type='relu',
            name="relu69_1_30_")

        conv70_1_30_ = mx.symbol.pad(data=relu69_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_30_ = mx.symbol.Convolution(data=conv70_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_30_")
        # conv70_1_30_, output shape: {[16,14,14]}

        batchnorm70_1_30_ = mx.symbol.BatchNorm(data=conv70_1_30_,
            fix_gamma=True,
            name="batchnorm70_1_30_")
        relu70_1_30_ = mx.symbol.Activation(data=batchnorm70_1_30_,
            act_type='relu',
            name="relu70_1_30_")

        conv71_1_30_ = mx.symbol.Convolution(data=relu70_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_30_")
        # conv71_1_30_, output shape: {[1024,14,14]}

        batchnorm71_1_30_ = mx.symbol.BatchNorm(data=conv71_1_30_,
            fix_gamma=True,
            name="batchnorm71_1_30_")
        conv69_1_31_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_31_")
        # conv69_1_31_, output shape: {[16,14,14]}

        batchnorm69_1_31_ = mx.symbol.BatchNorm(data=conv69_1_31_,
            fix_gamma=True,
            name="batchnorm69_1_31_")
        relu69_1_31_ = mx.symbol.Activation(data=batchnorm69_1_31_,
            act_type='relu',
            name="relu69_1_31_")

        conv70_1_31_ = mx.symbol.pad(data=relu69_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_31_ = mx.symbol.Convolution(data=conv70_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_31_")
        # conv70_1_31_, output shape: {[16,14,14]}

        batchnorm70_1_31_ = mx.symbol.BatchNorm(data=conv70_1_31_,
            fix_gamma=True,
            name="batchnorm70_1_31_")
        relu70_1_31_ = mx.symbol.Activation(data=batchnorm70_1_31_,
            act_type='relu',
            name="relu70_1_31_")

        conv71_1_31_ = mx.symbol.Convolution(data=relu70_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_31_")
        # conv71_1_31_, output shape: {[1024,14,14]}

        batchnorm71_1_31_ = mx.symbol.BatchNorm(data=conv71_1_31_,
            fix_gamma=True,
            name="batchnorm71_1_31_")
        conv69_1_32_ = mx.symbol.Convolution(data=relu67_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv69_1_32_")
        # conv69_1_32_, output shape: {[16,14,14]}

        batchnorm69_1_32_ = mx.symbol.BatchNorm(data=conv69_1_32_,
            fix_gamma=True,
            name="batchnorm69_1_32_")
        relu69_1_32_ = mx.symbol.Activation(data=batchnorm69_1_32_,
            act_type='relu',
            name="relu69_1_32_")

        conv70_1_32_ = mx.symbol.pad(data=relu69_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv70_1_32_ = mx.symbol.Convolution(data=conv70_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv70_1_32_")
        # conv70_1_32_, output shape: {[16,14,14]}

        batchnorm70_1_32_ = mx.symbol.BatchNorm(data=conv70_1_32_,
            fix_gamma=True,
            name="batchnorm70_1_32_")
        relu70_1_32_ = mx.symbol.Activation(data=batchnorm70_1_32_,
            act_type='relu',
            name="relu70_1_32_")

        conv71_1_32_ = mx.symbol.Convolution(data=relu70_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv71_1_32_")
        # conv71_1_32_, output shape: {[1024,14,14]}

        batchnorm71_1_32_ = mx.symbol.BatchNorm(data=conv71_1_32_,
            fix_gamma=True,
            name="batchnorm71_1_32_")
        add72_1_ = batchnorm71_1_1_ + batchnorm71_1_2_ + batchnorm71_1_3_ + batchnorm71_1_4_ + batchnorm71_1_5_ + batchnorm71_1_6_ + batchnorm71_1_7_ + batchnorm71_1_8_ + batchnorm71_1_9_ + batchnorm71_1_10_ + batchnorm71_1_11_ + batchnorm71_1_12_ + batchnorm71_1_13_ + batchnorm71_1_14_ + batchnorm71_1_15_ + batchnorm71_1_16_ + batchnorm71_1_17_ + batchnorm71_1_18_ + batchnorm71_1_19_ + batchnorm71_1_20_ + batchnorm71_1_21_ + batchnorm71_1_22_ + batchnorm71_1_23_ + batchnorm71_1_24_ + batchnorm71_1_25_ + batchnorm71_1_26_ + batchnorm71_1_27_ + batchnorm71_1_28_ + batchnorm71_1_29_ + batchnorm71_1_30_ + batchnorm71_1_31_ + batchnorm71_1_32_
        # add72_1_, output shape: {[1024,14,14]}

        add73_ = add72_1_ + relu67_
        # add73_, output shape: {[1024,14,14]}

        relu73_ = mx.symbol.Activation(data=add73_,
            act_type='relu',
            name="relu73_")

        conv75_1_1_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_1_")
        # conv75_1_1_, output shape: {[16,14,14]}

        batchnorm75_1_1_ = mx.symbol.BatchNorm(data=conv75_1_1_,
            fix_gamma=True,
            name="batchnorm75_1_1_")
        relu75_1_1_ = mx.symbol.Activation(data=batchnorm75_1_1_,
            act_type='relu',
            name="relu75_1_1_")

        conv76_1_1_ = mx.symbol.pad(data=relu75_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_1_ = mx.symbol.Convolution(data=conv76_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_1_")
        # conv76_1_1_, output shape: {[16,14,14]}

        batchnorm76_1_1_ = mx.symbol.BatchNorm(data=conv76_1_1_,
            fix_gamma=True,
            name="batchnorm76_1_1_")
        relu76_1_1_ = mx.symbol.Activation(data=batchnorm76_1_1_,
            act_type='relu',
            name="relu76_1_1_")

        conv77_1_1_ = mx.symbol.Convolution(data=relu76_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_1_")
        # conv77_1_1_, output shape: {[1024,14,14]}

        batchnorm77_1_1_ = mx.symbol.BatchNorm(data=conv77_1_1_,
            fix_gamma=True,
            name="batchnorm77_1_1_")
        conv75_1_2_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_2_")
        # conv75_1_2_, output shape: {[16,14,14]}

        batchnorm75_1_2_ = mx.symbol.BatchNorm(data=conv75_1_2_,
            fix_gamma=True,
            name="batchnorm75_1_2_")
        relu75_1_2_ = mx.symbol.Activation(data=batchnorm75_1_2_,
            act_type='relu',
            name="relu75_1_2_")

        conv76_1_2_ = mx.symbol.pad(data=relu75_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_2_ = mx.symbol.Convolution(data=conv76_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_2_")
        # conv76_1_2_, output shape: {[16,14,14]}

        batchnorm76_1_2_ = mx.symbol.BatchNorm(data=conv76_1_2_,
            fix_gamma=True,
            name="batchnorm76_1_2_")
        relu76_1_2_ = mx.symbol.Activation(data=batchnorm76_1_2_,
            act_type='relu',
            name="relu76_1_2_")

        conv77_1_2_ = mx.symbol.Convolution(data=relu76_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_2_")
        # conv77_1_2_, output shape: {[1024,14,14]}

        batchnorm77_1_2_ = mx.symbol.BatchNorm(data=conv77_1_2_,
            fix_gamma=True,
            name="batchnorm77_1_2_")
        conv75_1_3_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_3_")
        # conv75_1_3_, output shape: {[16,14,14]}

        batchnorm75_1_3_ = mx.symbol.BatchNorm(data=conv75_1_3_,
            fix_gamma=True,
            name="batchnorm75_1_3_")
        relu75_1_3_ = mx.symbol.Activation(data=batchnorm75_1_3_,
            act_type='relu',
            name="relu75_1_3_")

        conv76_1_3_ = mx.symbol.pad(data=relu75_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_3_ = mx.symbol.Convolution(data=conv76_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_3_")
        # conv76_1_3_, output shape: {[16,14,14]}

        batchnorm76_1_3_ = mx.symbol.BatchNorm(data=conv76_1_3_,
            fix_gamma=True,
            name="batchnorm76_1_3_")
        relu76_1_3_ = mx.symbol.Activation(data=batchnorm76_1_3_,
            act_type='relu',
            name="relu76_1_3_")

        conv77_1_3_ = mx.symbol.Convolution(data=relu76_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_3_")
        # conv77_1_3_, output shape: {[1024,14,14]}

        batchnorm77_1_3_ = mx.symbol.BatchNorm(data=conv77_1_3_,
            fix_gamma=True,
            name="batchnorm77_1_3_")
        conv75_1_4_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_4_")
        # conv75_1_4_, output shape: {[16,14,14]}

        batchnorm75_1_4_ = mx.symbol.BatchNorm(data=conv75_1_4_,
            fix_gamma=True,
            name="batchnorm75_1_4_")
        relu75_1_4_ = mx.symbol.Activation(data=batchnorm75_1_4_,
            act_type='relu',
            name="relu75_1_4_")

        conv76_1_4_ = mx.symbol.pad(data=relu75_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_4_ = mx.symbol.Convolution(data=conv76_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_4_")
        # conv76_1_4_, output shape: {[16,14,14]}

        batchnorm76_1_4_ = mx.symbol.BatchNorm(data=conv76_1_4_,
            fix_gamma=True,
            name="batchnorm76_1_4_")
        relu76_1_4_ = mx.symbol.Activation(data=batchnorm76_1_4_,
            act_type='relu',
            name="relu76_1_4_")

        conv77_1_4_ = mx.symbol.Convolution(data=relu76_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_4_")
        # conv77_1_4_, output shape: {[1024,14,14]}

        batchnorm77_1_4_ = mx.symbol.BatchNorm(data=conv77_1_4_,
            fix_gamma=True,
            name="batchnorm77_1_4_")
        conv75_1_5_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_5_")
        # conv75_1_5_, output shape: {[16,14,14]}

        batchnorm75_1_5_ = mx.symbol.BatchNorm(data=conv75_1_5_,
            fix_gamma=True,
            name="batchnorm75_1_5_")
        relu75_1_5_ = mx.symbol.Activation(data=batchnorm75_1_5_,
            act_type='relu',
            name="relu75_1_5_")

        conv76_1_5_ = mx.symbol.pad(data=relu75_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_5_ = mx.symbol.Convolution(data=conv76_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_5_")
        # conv76_1_5_, output shape: {[16,14,14]}

        batchnorm76_1_5_ = mx.symbol.BatchNorm(data=conv76_1_5_,
            fix_gamma=True,
            name="batchnorm76_1_5_")
        relu76_1_5_ = mx.symbol.Activation(data=batchnorm76_1_5_,
            act_type='relu',
            name="relu76_1_5_")

        conv77_1_5_ = mx.symbol.Convolution(data=relu76_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_5_")
        # conv77_1_5_, output shape: {[1024,14,14]}

        batchnorm77_1_5_ = mx.symbol.BatchNorm(data=conv77_1_5_,
            fix_gamma=True,
            name="batchnorm77_1_5_")
        conv75_1_6_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_6_")
        # conv75_1_6_, output shape: {[16,14,14]}

        batchnorm75_1_6_ = mx.symbol.BatchNorm(data=conv75_1_6_,
            fix_gamma=True,
            name="batchnorm75_1_6_")
        relu75_1_6_ = mx.symbol.Activation(data=batchnorm75_1_6_,
            act_type='relu',
            name="relu75_1_6_")

        conv76_1_6_ = mx.symbol.pad(data=relu75_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_6_ = mx.symbol.Convolution(data=conv76_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_6_")
        # conv76_1_6_, output shape: {[16,14,14]}

        batchnorm76_1_6_ = mx.symbol.BatchNorm(data=conv76_1_6_,
            fix_gamma=True,
            name="batchnorm76_1_6_")
        relu76_1_6_ = mx.symbol.Activation(data=batchnorm76_1_6_,
            act_type='relu',
            name="relu76_1_6_")

        conv77_1_6_ = mx.symbol.Convolution(data=relu76_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_6_")
        # conv77_1_6_, output shape: {[1024,14,14]}

        batchnorm77_1_6_ = mx.symbol.BatchNorm(data=conv77_1_6_,
            fix_gamma=True,
            name="batchnorm77_1_6_")
        conv75_1_7_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_7_")
        # conv75_1_7_, output shape: {[16,14,14]}

        batchnorm75_1_7_ = mx.symbol.BatchNorm(data=conv75_1_7_,
            fix_gamma=True,
            name="batchnorm75_1_7_")
        relu75_1_7_ = mx.symbol.Activation(data=batchnorm75_1_7_,
            act_type='relu',
            name="relu75_1_7_")

        conv76_1_7_ = mx.symbol.pad(data=relu75_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_7_ = mx.symbol.Convolution(data=conv76_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_7_")
        # conv76_1_7_, output shape: {[16,14,14]}

        batchnorm76_1_7_ = mx.symbol.BatchNorm(data=conv76_1_7_,
            fix_gamma=True,
            name="batchnorm76_1_7_")
        relu76_1_7_ = mx.symbol.Activation(data=batchnorm76_1_7_,
            act_type='relu',
            name="relu76_1_7_")

        conv77_1_7_ = mx.symbol.Convolution(data=relu76_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_7_")
        # conv77_1_7_, output shape: {[1024,14,14]}

        batchnorm77_1_7_ = mx.symbol.BatchNorm(data=conv77_1_7_,
            fix_gamma=True,
            name="batchnorm77_1_7_")
        conv75_1_8_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_8_")
        # conv75_1_8_, output shape: {[16,14,14]}

        batchnorm75_1_8_ = mx.symbol.BatchNorm(data=conv75_1_8_,
            fix_gamma=True,
            name="batchnorm75_1_8_")
        relu75_1_8_ = mx.symbol.Activation(data=batchnorm75_1_8_,
            act_type='relu',
            name="relu75_1_8_")

        conv76_1_8_ = mx.symbol.pad(data=relu75_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_8_ = mx.symbol.Convolution(data=conv76_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_8_")
        # conv76_1_8_, output shape: {[16,14,14]}

        batchnorm76_1_8_ = mx.symbol.BatchNorm(data=conv76_1_8_,
            fix_gamma=True,
            name="batchnorm76_1_8_")
        relu76_1_8_ = mx.symbol.Activation(data=batchnorm76_1_8_,
            act_type='relu',
            name="relu76_1_8_")

        conv77_1_8_ = mx.symbol.Convolution(data=relu76_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_8_")
        # conv77_1_8_, output shape: {[1024,14,14]}

        batchnorm77_1_8_ = mx.symbol.BatchNorm(data=conv77_1_8_,
            fix_gamma=True,
            name="batchnorm77_1_8_")
        conv75_1_9_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_9_")
        # conv75_1_9_, output shape: {[16,14,14]}

        batchnorm75_1_9_ = mx.symbol.BatchNorm(data=conv75_1_9_,
            fix_gamma=True,
            name="batchnorm75_1_9_")
        relu75_1_9_ = mx.symbol.Activation(data=batchnorm75_1_9_,
            act_type='relu',
            name="relu75_1_9_")

        conv76_1_9_ = mx.symbol.pad(data=relu75_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_9_ = mx.symbol.Convolution(data=conv76_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_9_")
        # conv76_1_9_, output shape: {[16,14,14]}

        batchnorm76_1_9_ = mx.symbol.BatchNorm(data=conv76_1_9_,
            fix_gamma=True,
            name="batchnorm76_1_9_")
        relu76_1_9_ = mx.symbol.Activation(data=batchnorm76_1_9_,
            act_type='relu',
            name="relu76_1_9_")

        conv77_1_9_ = mx.symbol.Convolution(data=relu76_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_9_")
        # conv77_1_9_, output shape: {[1024,14,14]}

        batchnorm77_1_9_ = mx.symbol.BatchNorm(data=conv77_1_9_,
            fix_gamma=True,
            name="batchnorm77_1_9_")
        conv75_1_10_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_10_")
        # conv75_1_10_, output shape: {[16,14,14]}

        batchnorm75_1_10_ = mx.symbol.BatchNorm(data=conv75_1_10_,
            fix_gamma=True,
            name="batchnorm75_1_10_")
        relu75_1_10_ = mx.symbol.Activation(data=batchnorm75_1_10_,
            act_type='relu',
            name="relu75_1_10_")

        conv76_1_10_ = mx.symbol.pad(data=relu75_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_10_ = mx.symbol.Convolution(data=conv76_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_10_")
        # conv76_1_10_, output shape: {[16,14,14]}

        batchnorm76_1_10_ = mx.symbol.BatchNorm(data=conv76_1_10_,
            fix_gamma=True,
            name="batchnorm76_1_10_")
        relu76_1_10_ = mx.symbol.Activation(data=batchnorm76_1_10_,
            act_type='relu',
            name="relu76_1_10_")

        conv77_1_10_ = mx.symbol.Convolution(data=relu76_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_10_")
        # conv77_1_10_, output shape: {[1024,14,14]}

        batchnorm77_1_10_ = mx.symbol.BatchNorm(data=conv77_1_10_,
            fix_gamma=True,
            name="batchnorm77_1_10_")
        conv75_1_11_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_11_")
        # conv75_1_11_, output shape: {[16,14,14]}

        batchnorm75_1_11_ = mx.symbol.BatchNorm(data=conv75_1_11_,
            fix_gamma=True,
            name="batchnorm75_1_11_")
        relu75_1_11_ = mx.symbol.Activation(data=batchnorm75_1_11_,
            act_type='relu',
            name="relu75_1_11_")

        conv76_1_11_ = mx.symbol.pad(data=relu75_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_11_ = mx.symbol.Convolution(data=conv76_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_11_")
        # conv76_1_11_, output shape: {[16,14,14]}

        batchnorm76_1_11_ = mx.symbol.BatchNorm(data=conv76_1_11_,
            fix_gamma=True,
            name="batchnorm76_1_11_")
        relu76_1_11_ = mx.symbol.Activation(data=batchnorm76_1_11_,
            act_type='relu',
            name="relu76_1_11_")

        conv77_1_11_ = mx.symbol.Convolution(data=relu76_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_11_")
        # conv77_1_11_, output shape: {[1024,14,14]}

        batchnorm77_1_11_ = mx.symbol.BatchNorm(data=conv77_1_11_,
            fix_gamma=True,
            name="batchnorm77_1_11_")
        conv75_1_12_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_12_")
        # conv75_1_12_, output shape: {[16,14,14]}

        batchnorm75_1_12_ = mx.symbol.BatchNorm(data=conv75_1_12_,
            fix_gamma=True,
            name="batchnorm75_1_12_")
        relu75_1_12_ = mx.symbol.Activation(data=batchnorm75_1_12_,
            act_type='relu',
            name="relu75_1_12_")

        conv76_1_12_ = mx.symbol.pad(data=relu75_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_12_ = mx.symbol.Convolution(data=conv76_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_12_")
        # conv76_1_12_, output shape: {[16,14,14]}

        batchnorm76_1_12_ = mx.symbol.BatchNorm(data=conv76_1_12_,
            fix_gamma=True,
            name="batchnorm76_1_12_")
        relu76_1_12_ = mx.symbol.Activation(data=batchnorm76_1_12_,
            act_type='relu',
            name="relu76_1_12_")

        conv77_1_12_ = mx.symbol.Convolution(data=relu76_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_12_")
        # conv77_1_12_, output shape: {[1024,14,14]}

        batchnorm77_1_12_ = mx.symbol.BatchNorm(data=conv77_1_12_,
            fix_gamma=True,
            name="batchnorm77_1_12_")
        conv75_1_13_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_13_")
        # conv75_1_13_, output shape: {[16,14,14]}

        batchnorm75_1_13_ = mx.symbol.BatchNorm(data=conv75_1_13_,
            fix_gamma=True,
            name="batchnorm75_1_13_")
        relu75_1_13_ = mx.symbol.Activation(data=batchnorm75_1_13_,
            act_type='relu',
            name="relu75_1_13_")

        conv76_1_13_ = mx.symbol.pad(data=relu75_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_13_ = mx.symbol.Convolution(data=conv76_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_13_")
        # conv76_1_13_, output shape: {[16,14,14]}

        batchnorm76_1_13_ = mx.symbol.BatchNorm(data=conv76_1_13_,
            fix_gamma=True,
            name="batchnorm76_1_13_")
        relu76_1_13_ = mx.symbol.Activation(data=batchnorm76_1_13_,
            act_type='relu',
            name="relu76_1_13_")

        conv77_1_13_ = mx.symbol.Convolution(data=relu76_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_13_")
        # conv77_1_13_, output shape: {[1024,14,14]}

        batchnorm77_1_13_ = mx.symbol.BatchNorm(data=conv77_1_13_,
            fix_gamma=True,
            name="batchnorm77_1_13_")
        conv75_1_14_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_14_")
        # conv75_1_14_, output shape: {[16,14,14]}

        batchnorm75_1_14_ = mx.symbol.BatchNorm(data=conv75_1_14_,
            fix_gamma=True,
            name="batchnorm75_1_14_")
        relu75_1_14_ = mx.symbol.Activation(data=batchnorm75_1_14_,
            act_type='relu',
            name="relu75_1_14_")

        conv76_1_14_ = mx.symbol.pad(data=relu75_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_14_ = mx.symbol.Convolution(data=conv76_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_14_")
        # conv76_1_14_, output shape: {[16,14,14]}

        batchnorm76_1_14_ = mx.symbol.BatchNorm(data=conv76_1_14_,
            fix_gamma=True,
            name="batchnorm76_1_14_")
        relu76_1_14_ = mx.symbol.Activation(data=batchnorm76_1_14_,
            act_type='relu',
            name="relu76_1_14_")

        conv77_1_14_ = mx.symbol.Convolution(data=relu76_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_14_")
        # conv77_1_14_, output shape: {[1024,14,14]}

        batchnorm77_1_14_ = mx.symbol.BatchNorm(data=conv77_1_14_,
            fix_gamma=True,
            name="batchnorm77_1_14_")
        conv75_1_15_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_15_")
        # conv75_1_15_, output shape: {[16,14,14]}

        batchnorm75_1_15_ = mx.symbol.BatchNorm(data=conv75_1_15_,
            fix_gamma=True,
            name="batchnorm75_1_15_")
        relu75_1_15_ = mx.symbol.Activation(data=batchnorm75_1_15_,
            act_type='relu',
            name="relu75_1_15_")

        conv76_1_15_ = mx.symbol.pad(data=relu75_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_15_ = mx.symbol.Convolution(data=conv76_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_15_")
        # conv76_1_15_, output shape: {[16,14,14]}

        batchnorm76_1_15_ = mx.symbol.BatchNorm(data=conv76_1_15_,
            fix_gamma=True,
            name="batchnorm76_1_15_")
        relu76_1_15_ = mx.symbol.Activation(data=batchnorm76_1_15_,
            act_type='relu',
            name="relu76_1_15_")

        conv77_1_15_ = mx.symbol.Convolution(data=relu76_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_15_")
        # conv77_1_15_, output shape: {[1024,14,14]}

        batchnorm77_1_15_ = mx.symbol.BatchNorm(data=conv77_1_15_,
            fix_gamma=True,
            name="batchnorm77_1_15_")
        conv75_1_16_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_16_")
        # conv75_1_16_, output shape: {[16,14,14]}

        batchnorm75_1_16_ = mx.symbol.BatchNorm(data=conv75_1_16_,
            fix_gamma=True,
            name="batchnorm75_1_16_")
        relu75_1_16_ = mx.symbol.Activation(data=batchnorm75_1_16_,
            act_type='relu',
            name="relu75_1_16_")

        conv76_1_16_ = mx.symbol.pad(data=relu75_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_16_ = mx.symbol.Convolution(data=conv76_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_16_")
        # conv76_1_16_, output shape: {[16,14,14]}

        batchnorm76_1_16_ = mx.symbol.BatchNorm(data=conv76_1_16_,
            fix_gamma=True,
            name="batchnorm76_1_16_")
        relu76_1_16_ = mx.symbol.Activation(data=batchnorm76_1_16_,
            act_type='relu',
            name="relu76_1_16_")

        conv77_1_16_ = mx.symbol.Convolution(data=relu76_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_16_")
        # conv77_1_16_, output shape: {[1024,14,14]}

        batchnorm77_1_16_ = mx.symbol.BatchNorm(data=conv77_1_16_,
            fix_gamma=True,
            name="batchnorm77_1_16_")
        conv75_1_17_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_17_")
        # conv75_1_17_, output shape: {[16,14,14]}

        batchnorm75_1_17_ = mx.symbol.BatchNorm(data=conv75_1_17_,
            fix_gamma=True,
            name="batchnorm75_1_17_")
        relu75_1_17_ = mx.symbol.Activation(data=batchnorm75_1_17_,
            act_type='relu',
            name="relu75_1_17_")

        conv76_1_17_ = mx.symbol.pad(data=relu75_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_17_ = mx.symbol.Convolution(data=conv76_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_17_")
        # conv76_1_17_, output shape: {[16,14,14]}

        batchnorm76_1_17_ = mx.symbol.BatchNorm(data=conv76_1_17_,
            fix_gamma=True,
            name="batchnorm76_1_17_")
        relu76_1_17_ = mx.symbol.Activation(data=batchnorm76_1_17_,
            act_type='relu',
            name="relu76_1_17_")

        conv77_1_17_ = mx.symbol.Convolution(data=relu76_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_17_")
        # conv77_1_17_, output shape: {[1024,14,14]}

        batchnorm77_1_17_ = mx.symbol.BatchNorm(data=conv77_1_17_,
            fix_gamma=True,
            name="batchnorm77_1_17_")
        conv75_1_18_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_18_")
        # conv75_1_18_, output shape: {[16,14,14]}

        batchnorm75_1_18_ = mx.symbol.BatchNorm(data=conv75_1_18_,
            fix_gamma=True,
            name="batchnorm75_1_18_")
        relu75_1_18_ = mx.symbol.Activation(data=batchnorm75_1_18_,
            act_type='relu',
            name="relu75_1_18_")

        conv76_1_18_ = mx.symbol.pad(data=relu75_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_18_ = mx.symbol.Convolution(data=conv76_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_18_")
        # conv76_1_18_, output shape: {[16,14,14]}

        batchnorm76_1_18_ = mx.symbol.BatchNorm(data=conv76_1_18_,
            fix_gamma=True,
            name="batchnorm76_1_18_")
        relu76_1_18_ = mx.symbol.Activation(data=batchnorm76_1_18_,
            act_type='relu',
            name="relu76_1_18_")

        conv77_1_18_ = mx.symbol.Convolution(data=relu76_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_18_")
        # conv77_1_18_, output shape: {[1024,14,14]}

        batchnorm77_1_18_ = mx.symbol.BatchNorm(data=conv77_1_18_,
            fix_gamma=True,
            name="batchnorm77_1_18_")
        conv75_1_19_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_19_")
        # conv75_1_19_, output shape: {[16,14,14]}

        batchnorm75_1_19_ = mx.symbol.BatchNorm(data=conv75_1_19_,
            fix_gamma=True,
            name="batchnorm75_1_19_")
        relu75_1_19_ = mx.symbol.Activation(data=batchnorm75_1_19_,
            act_type='relu',
            name="relu75_1_19_")

        conv76_1_19_ = mx.symbol.pad(data=relu75_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_19_ = mx.symbol.Convolution(data=conv76_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_19_")
        # conv76_1_19_, output shape: {[16,14,14]}

        batchnorm76_1_19_ = mx.symbol.BatchNorm(data=conv76_1_19_,
            fix_gamma=True,
            name="batchnorm76_1_19_")
        relu76_1_19_ = mx.symbol.Activation(data=batchnorm76_1_19_,
            act_type='relu',
            name="relu76_1_19_")

        conv77_1_19_ = mx.symbol.Convolution(data=relu76_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_19_")
        # conv77_1_19_, output shape: {[1024,14,14]}

        batchnorm77_1_19_ = mx.symbol.BatchNorm(data=conv77_1_19_,
            fix_gamma=True,
            name="batchnorm77_1_19_")
        conv75_1_20_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_20_")
        # conv75_1_20_, output shape: {[16,14,14]}

        batchnorm75_1_20_ = mx.symbol.BatchNorm(data=conv75_1_20_,
            fix_gamma=True,
            name="batchnorm75_1_20_")
        relu75_1_20_ = mx.symbol.Activation(data=batchnorm75_1_20_,
            act_type='relu',
            name="relu75_1_20_")

        conv76_1_20_ = mx.symbol.pad(data=relu75_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_20_ = mx.symbol.Convolution(data=conv76_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_20_")
        # conv76_1_20_, output shape: {[16,14,14]}

        batchnorm76_1_20_ = mx.symbol.BatchNorm(data=conv76_1_20_,
            fix_gamma=True,
            name="batchnorm76_1_20_")
        relu76_1_20_ = mx.symbol.Activation(data=batchnorm76_1_20_,
            act_type='relu',
            name="relu76_1_20_")

        conv77_1_20_ = mx.symbol.Convolution(data=relu76_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_20_")
        # conv77_1_20_, output shape: {[1024,14,14]}

        batchnorm77_1_20_ = mx.symbol.BatchNorm(data=conv77_1_20_,
            fix_gamma=True,
            name="batchnorm77_1_20_")
        conv75_1_21_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_21_")
        # conv75_1_21_, output shape: {[16,14,14]}

        batchnorm75_1_21_ = mx.symbol.BatchNorm(data=conv75_1_21_,
            fix_gamma=True,
            name="batchnorm75_1_21_")
        relu75_1_21_ = mx.symbol.Activation(data=batchnorm75_1_21_,
            act_type='relu',
            name="relu75_1_21_")

        conv76_1_21_ = mx.symbol.pad(data=relu75_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_21_ = mx.symbol.Convolution(data=conv76_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_21_")
        # conv76_1_21_, output shape: {[16,14,14]}

        batchnorm76_1_21_ = mx.symbol.BatchNorm(data=conv76_1_21_,
            fix_gamma=True,
            name="batchnorm76_1_21_")
        relu76_1_21_ = mx.symbol.Activation(data=batchnorm76_1_21_,
            act_type='relu',
            name="relu76_1_21_")

        conv77_1_21_ = mx.symbol.Convolution(data=relu76_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_21_")
        # conv77_1_21_, output shape: {[1024,14,14]}

        batchnorm77_1_21_ = mx.symbol.BatchNorm(data=conv77_1_21_,
            fix_gamma=True,
            name="batchnorm77_1_21_")
        conv75_1_22_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_22_")
        # conv75_1_22_, output shape: {[16,14,14]}

        batchnorm75_1_22_ = mx.symbol.BatchNorm(data=conv75_1_22_,
            fix_gamma=True,
            name="batchnorm75_1_22_")
        relu75_1_22_ = mx.symbol.Activation(data=batchnorm75_1_22_,
            act_type='relu',
            name="relu75_1_22_")

        conv76_1_22_ = mx.symbol.pad(data=relu75_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_22_ = mx.symbol.Convolution(data=conv76_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_22_")
        # conv76_1_22_, output shape: {[16,14,14]}

        batchnorm76_1_22_ = mx.symbol.BatchNorm(data=conv76_1_22_,
            fix_gamma=True,
            name="batchnorm76_1_22_")
        relu76_1_22_ = mx.symbol.Activation(data=batchnorm76_1_22_,
            act_type='relu',
            name="relu76_1_22_")

        conv77_1_22_ = mx.symbol.Convolution(data=relu76_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_22_")
        # conv77_1_22_, output shape: {[1024,14,14]}

        batchnorm77_1_22_ = mx.symbol.BatchNorm(data=conv77_1_22_,
            fix_gamma=True,
            name="batchnorm77_1_22_")
        conv75_1_23_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_23_")
        # conv75_1_23_, output shape: {[16,14,14]}

        batchnorm75_1_23_ = mx.symbol.BatchNorm(data=conv75_1_23_,
            fix_gamma=True,
            name="batchnorm75_1_23_")
        relu75_1_23_ = mx.symbol.Activation(data=batchnorm75_1_23_,
            act_type='relu',
            name="relu75_1_23_")

        conv76_1_23_ = mx.symbol.pad(data=relu75_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_23_ = mx.symbol.Convolution(data=conv76_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_23_")
        # conv76_1_23_, output shape: {[16,14,14]}

        batchnorm76_1_23_ = mx.symbol.BatchNorm(data=conv76_1_23_,
            fix_gamma=True,
            name="batchnorm76_1_23_")
        relu76_1_23_ = mx.symbol.Activation(data=batchnorm76_1_23_,
            act_type='relu',
            name="relu76_1_23_")

        conv77_1_23_ = mx.symbol.Convolution(data=relu76_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_23_")
        # conv77_1_23_, output shape: {[1024,14,14]}

        batchnorm77_1_23_ = mx.symbol.BatchNorm(data=conv77_1_23_,
            fix_gamma=True,
            name="batchnorm77_1_23_")
        conv75_1_24_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_24_")
        # conv75_1_24_, output shape: {[16,14,14]}

        batchnorm75_1_24_ = mx.symbol.BatchNorm(data=conv75_1_24_,
            fix_gamma=True,
            name="batchnorm75_1_24_")
        relu75_1_24_ = mx.symbol.Activation(data=batchnorm75_1_24_,
            act_type='relu',
            name="relu75_1_24_")

        conv76_1_24_ = mx.symbol.pad(data=relu75_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_24_ = mx.symbol.Convolution(data=conv76_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_24_")
        # conv76_1_24_, output shape: {[16,14,14]}

        batchnorm76_1_24_ = mx.symbol.BatchNorm(data=conv76_1_24_,
            fix_gamma=True,
            name="batchnorm76_1_24_")
        relu76_1_24_ = mx.symbol.Activation(data=batchnorm76_1_24_,
            act_type='relu',
            name="relu76_1_24_")

        conv77_1_24_ = mx.symbol.Convolution(data=relu76_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_24_")
        # conv77_1_24_, output shape: {[1024,14,14]}

        batchnorm77_1_24_ = mx.symbol.BatchNorm(data=conv77_1_24_,
            fix_gamma=True,
            name="batchnorm77_1_24_")
        conv75_1_25_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_25_")
        # conv75_1_25_, output shape: {[16,14,14]}

        batchnorm75_1_25_ = mx.symbol.BatchNorm(data=conv75_1_25_,
            fix_gamma=True,
            name="batchnorm75_1_25_")
        relu75_1_25_ = mx.symbol.Activation(data=batchnorm75_1_25_,
            act_type='relu',
            name="relu75_1_25_")

        conv76_1_25_ = mx.symbol.pad(data=relu75_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_25_ = mx.symbol.Convolution(data=conv76_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_25_")
        # conv76_1_25_, output shape: {[16,14,14]}

        batchnorm76_1_25_ = mx.symbol.BatchNorm(data=conv76_1_25_,
            fix_gamma=True,
            name="batchnorm76_1_25_")
        relu76_1_25_ = mx.symbol.Activation(data=batchnorm76_1_25_,
            act_type='relu',
            name="relu76_1_25_")

        conv77_1_25_ = mx.symbol.Convolution(data=relu76_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_25_")
        # conv77_1_25_, output shape: {[1024,14,14]}

        batchnorm77_1_25_ = mx.symbol.BatchNorm(data=conv77_1_25_,
            fix_gamma=True,
            name="batchnorm77_1_25_")
        conv75_1_26_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_26_")
        # conv75_1_26_, output shape: {[16,14,14]}

        batchnorm75_1_26_ = mx.symbol.BatchNorm(data=conv75_1_26_,
            fix_gamma=True,
            name="batchnorm75_1_26_")
        relu75_1_26_ = mx.symbol.Activation(data=batchnorm75_1_26_,
            act_type='relu',
            name="relu75_1_26_")

        conv76_1_26_ = mx.symbol.pad(data=relu75_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_26_ = mx.symbol.Convolution(data=conv76_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_26_")
        # conv76_1_26_, output shape: {[16,14,14]}

        batchnorm76_1_26_ = mx.symbol.BatchNorm(data=conv76_1_26_,
            fix_gamma=True,
            name="batchnorm76_1_26_")
        relu76_1_26_ = mx.symbol.Activation(data=batchnorm76_1_26_,
            act_type='relu',
            name="relu76_1_26_")

        conv77_1_26_ = mx.symbol.Convolution(data=relu76_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_26_")
        # conv77_1_26_, output shape: {[1024,14,14]}

        batchnorm77_1_26_ = mx.symbol.BatchNorm(data=conv77_1_26_,
            fix_gamma=True,
            name="batchnorm77_1_26_")
        conv75_1_27_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_27_")
        # conv75_1_27_, output shape: {[16,14,14]}

        batchnorm75_1_27_ = mx.symbol.BatchNorm(data=conv75_1_27_,
            fix_gamma=True,
            name="batchnorm75_1_27_")
        relu75_1_27_ = mx.symbol.Activation(data=batchnorm75_1_27_,
            act_type='relu',
            name="relu75_1_27_")

        conv76_1_27_ = mx.symbol.pad(data=relu75_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_27_ = mx.symbol.Convolution(data=conv76_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_27_")
        # conv76_1_27_, output shape: {[16,14,14]}

        batchnorm76_1_27_ = mx.symbol.BatchNorm(data=conv76_1_27_,
            fix_gamma=True,
            name="batchnorm76_1_27_")
        relu76_1_27_ = mx.symbol.Activation(data=batchnorm76_1_27_,
            act_type='relu',
            name="relu76_1_27_")

        conv77_1_27_ = mx.symbol.Convolution(data=relu76_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_27_")
        # conv77_1_27_, output shape: {[1024,14,14]}

        batchnorm77_1_27_ = mx.symbol.BatchNorm(data=conv77_1_27_,
            fix_gamma=True,
            name="batchnorm77_1_27_")
        conv75_1_28_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_28_")
        # conv75_1_28_, output shape: {[16,14,14]}

        batchnorm75_1_28_ = mx.symbol.BatchNorm(data=conv75_1_28_,
            fix_gamma=True,
            name="batchnorm75_1_28_")
        relu75_1_28_ = mx.symbol.Activation(data=batchnorm75_1_28_,
            act_type='relu',
            name="relu75_1_28_")

        conv76_1_28_ = mx.symbol.pad(data=relu75_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_28_ = mx.symbol.Convolution(data=conv76_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_28_")
        # conv76_1_28_, output shape: {[16,14,14]}

        batchnorm76_1_28_ = mx.symbol.BatchNorm(data=conv76_1_28_,
            fix_gamma=True,
            name="batchnorm76_1_28_")
        relu76_1_28_ = mx.symbol.Activation(data=batchnorm76_1_28_,
            act_type='relu',
            name="relu76_1_28_")

        conv77_1_28_ = mx.symbol.Convolution(data=relu76_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_28_")
        # conv77_1_28_, output shape: {[1024,14,14]}

        batchnorm77_1_28_ = mx.symbol.BatchNorm(data=conv77_1_28_,
            fix_gamma=True,
            name="batchnorm77_1_28_")
        conv75_1_29_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_29_")
        # conv75_1_29_, output shape: {[16,14,14]}

        batchnorm75_1_29_ = mx.symbol.BatchNorm(data=conv75_1_29_,
            fix_gamma=True,
            name="batchnorm75_1_29_")
        relu75_1_29_ = mx.symbol.Activation(data=batchnorm75_1_29_,
            act_type='relu',
            name="relu75_1_29_")

        conv76_1_29_ = mx.symbol.pad(data=relu75_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_29_ = mx.symbol.Convolution(data=conv76_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_29_")
        # conv76_1_29_, output shape: {[16,14,14]}

        batchnorm76_1_29_ = mx.symbol.BatchNorm(data=conv76_1_29_,
            fix_gamma=True,
            name="batchnorm76_1_29_")
        relu76_1_29_ = mx.symbol.Activation(data=batchnorm76_1_29_,
            act_type='relu',
            name="relu76_1_29_")

        conv77_1_29_ = mx.symbol.Convolution(data=relu76_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_29_")
        # conv77_1_29_, output shape: {[1024,14,14]}

        batchnorm77_1_29_ = mx.symbol.BatchNorm(data=conv77_1_29_,
            fix_gamma=True,
            name="batchnorm77_1_29_")
        conv75_1_30_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_30_")
        # conv75_1_30_, output shape: {[16,14,14]}

        batchnorm75_1_30_ = mx.symbol.BatchNorm(data=conv75_1_30_,
            fix_gamma=True,
            name="batchnorm75_1_30_")
        relu75_1_30_ = mx.symbol.Activation(data=batchnorm75_1_30_,
            act_type='relu',
            name="relu75_1_30_")

        conv76_1_30_ = mx.symbol.pad(data=relu75_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_30_ = mx.symbol.Convolution(data=conv76_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_30_")
        # conv76_1_30_, output shape: {[16,14,14]}

        batchnorm76_1_30_ = mx.symbol.BatchNorm(data=conv76_1_30_,
            fix_gamma=True,
            name="batchnorm76_1_30_")
        relu76_1_30_ = mx.symbol.Activation(data=batchnorm76_1_30_,
            act_type='relu',
            name="relu76_1_30_")

        conv77_1_30_ = mx.symbol.Convolution(data=relu76_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_30_")
        # conv77_1_30_, output shape: {[1024,14,14]}

        batchnorm77_1_30_ = mx.symbol.BatchNorm(data=conv77_1_30_,
            fix_gamma=True,
            name="batchnorm77_1_30_")
        conv75_1_31_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_31_")
        # conv75_1_31_, output shape: {[16,14,14]}

        batchnorm75_1_31_ = mx.symbol.BatchNorm(data=conv75_1_31_,
            fix_gamma=True,
            name="batchnorm75_1_31_")
        relu75_1_31_ = mx.symbol.Activation(data=batchnorm75_1_31_,
            act_type='relu',
            name="relu75_1_31_")

        conv76_1_31_ = mx.symbol.pad(data=relu75_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_31_ = mx.symbol.Convolution(data=conv76_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_31_")
        # conv76_1_31_, output shape: {[16,14,14]}

        batchnorm76_1_31_ = mx.symbol.BatchNorm(data=conv76_1_31_,
            fix_gamma=True,
            name="batchnorm76_1_31_")
        relu76_1_31_ = mx.symbol.Activation(data=batchnorm76_1_31_,
            act_type='relu',
            name="relu76_1_31_")

        conv77_1_31_ = mx.symbol.Convolution(data=relu76_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_31_")
        # conv77_1_31_, output shape: {[1024,14,14]}

        batchnorm77_1_31_ = mx.symbol.BatchNorm(data=conv77_1_31_,
            fix_gamma=True,
            name="batchnorm77_1_31_")
        conv75_1_32_ = mx.symbol.Convolution(data=relu73_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv75_1_32_")
        # conv75_1_32_, output shape: {[16,14,14]}

        batchnorm75_1_32_ = mx.symbol.BatchNorm(data=conv75_1_32_,
            fix_gamma=True,
            name="batchnorm75_1_32_")
        relu75_1_32_ = mx.symbol.Activation(data=batchnorm75_1_32_,
            act_type='relu',
            name="relu75_1_32_")

        conv76_1_32_ = mx.symbol.pad(data=relu75_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv76_1_32_ = mx.symbol.Convolution(data=conv76_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=16,
            no_bias=False,
            name="conv76_1_32_")
        # conv76_1_32_, output shape: {[16,14,14]}

        batchnorm76_1_32_ = mx.symbol.BatchNorm(data=conv76_1_32_,
            fix_gamma=True,
            name="batchnorm76_1_32_")
        relu76_1_32_ = mx.symbol.Activation(data=batchnorm76_1_32_,
            act_type='relu',
            name="relu76_1_32_")

        conv77_1_32_ = mx.symbol.Convolution(data=relu76_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=1024,
            no_bias=False,
            name="conv77_1_32_")
        # conv77_1_32_, output shape: {[1024,14,14]}

        batchnorm77_1_32_ = mx.symbol.BatchNorm(data=conv77_1_32_,
            fix_gamma=True,
            name="batchnorm77_1_32_")
        add78_1_ = batchnorm77_1_1_ + batchnorm77_1_2_ + batchnorm77_1_3_ + batchnorm77_1_4_ + batchnorm77_1_5_ + batchnorm77_1_6_ + batchnorm77_1_7_ + batchnorm77_1_8_ + batchnorm77_1_9_ + batchnorm77_1_10_ + batchnorm77_1_11_ + batchnorm77_1_12_ + batchnorm77_1_13_ + batchnorm77_1_14_ + batchnorm77_1_15_ + batchnorm77_1_16_ + batchnorm77_1_17_ + batchnorm77_1_18_ + batchnorm77_1_19_ + batchnorm77_1_20_ + batchnorm77_1_21_ + batchnorm77_1_22_ + batchnorm77_1_23_ + batchnorm77_1_24_ + batchnorm77_1_25_ + batchnorm77_1_26_ + batchnorm77_1_27_ + batchnorm77_1_28_ + batchnorm77_1_29_ + batchnorm77_1_30_ + batchnorm77_1_31_ + batchnorm77_1_32_
        # add78_1_, output shape: {[1024,14,14]}

        add79_ = add78_1_ + relu73_
        # add79_, output shape: {[1024,14,14]}

        relu79_ = mx.symbol.Activation(data=add79_,
            act_type='relu',
            name="relu79_")

        conv81_1_1_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_1_")
        # conv81_1_1_, output shape: {[32,14,14]}

        batchnorm81_1_1_ = mx.symbol.BatchNorm(data=conv81_1_1_,
            fix_gamma=True,
            name="batchnorm81_1_1_")
        relu81_1_1_ = mx.symbol.Activation(data=batchnorm81_1_1_,
            act_type='relu',
            name="relu81_1_1_")

        conv82_1_1_ = mx.symbol.pad(data=relu81_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_1_ = mx.symbol.Convolution(data=conv82_1_1_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_1_")
        # conv82_1_1_, output shape: {[32,7,7]}

        batchnorm82_1_1_ = mx.symbol.BatchNorm(data=conv82_1_1_,
            fix_gamma=True,
            name="batchnorm82_1_1_")
        relu82_1_1_ = mx.symbol.Activation(data=batchnorm82_1_1_,
            act_type='relu',
            name="relu82_1_1_")

        conv83_1_1_ = mx.symbol.Convolution(data=relu82_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_1_")
        # conv83_1_1_, output shape: {[2048,7,7]}

        batchnorm83_1_1_ = mx.symbol.BatchNorm(data=conv83_1_1_,
            fix_gamma=True,
            name="batchnorm83_1_1_")
        conv81_1_2_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_2_")
        # conv81_1_2_, output shape: {[32,14,14]}

        batchnorm81_1_2_ = mx.symbol.BatchNorm(data=conv81_1_2_,
            fix_gamma=True,
            name="batchnorm81_1_2_")
        relu81_1_2_ = mx.symbol.Activation(data=batchnorm81_1_2_,
            act_type='relu',
            name="relu81_1_2_")

        conv82_1_2_ = mx.symbol.pad(data=relu81_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_2_ = mx.symbol.Convolution(data=conv82_1_2_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_2_")
        # conv82_1_2_, output shape: {[32,7,7]}

        batchnorm82_1_2_ = mx.symbol.BatchNorm(data=conv82_1_2_,
            fix_gamma=True,
            name="batchnorm82_1_2_")
        relu82_1_2_ = mx.symbol.Activation(data=batchnorm82_1_2_,
            act_type='relu',
            name="relu82_1_2_")

        conv83_1_2_ = mx.symbol.Convolution(data=relu82_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_2_")
        # conv83_1_2_, output shape: {[2048,7,7]}

        batchnorm83_1_2_ = mx.symbol.BatchNorm(data=conv83_1_2_,
            fix_gamma=True,
            name="batchnorm83_1_2_")
        conv81_1_3_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_3_")
        # conv81_1_3_, output shape: {[32,14,14]}

        batchnorm81_1_3_ = mx.symbol.BatchNorm(data=conv81_1_3_,
            fix_gamma=True,
            name="batchnorm81_1_3_")
        relu81_1_3_ = mx.symbol.Activation(data=batchnorm81_1_3_,
            act_type='relu',
            name="relu81_1_3_")

        conv82_1_3_ = mx.symbol.pad(data=relu81_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_3_ = mx.symbol.Convolution(data=conv82_1_3_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_3_")
        # conv82_1_3_, output shape: {[32,7,7]}

        batchnorm82_1_3_ = mx.symbol.BatchNorm(data=conv82_1_3_,
            fix_gamma=True,
            name="batchnorm82_1_3_")
        relu82_1_3_ = mx.symbol.Activation(data=batchnorm82_1_3_,
            act_type='relu',
            name="relu82_1_3_")

        conv83_1_3_ = mx.symbol.Convolution(data=relu82_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_3_")
        # conv83_1_3_, output shape: {[2048,7,7]}

        batchnorm83_1_3_ = mx.symbol.BatchNorm(data=conv83_1_3_,
            fix_gamma=True,
            name="batchnorm83_1_3_")
        conv81_1_4_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_4_")
        # conv81_1_4_, output shape: {[32,14,14]}

        batchnorm81_1_4_ = mx.symbol.BatchNorm(data=conv81_1_4_,
            fix_gamma=True,
            name="batchnorm81_1_4_")
        relu81_1_4_ = mx.symbol.Activation(data=batchnorm81_1_4_,
            act_type='relu',
            name="relu81_1_4_")

        conv82_1_4_ = mx.symbol.pad(data=relu81_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_4_ = mx.symbol.Convolution(data=conv82_1_4_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_4_")
        # conv82_1_4_, output shape: {[32,7,7]}

        batchnorm82_1_4_ = mx.symbol.BatchNorm(data=conv82_1_4_,
            fix_gamma=True,
            name="batchnorm82_1_4_")
        relu82_1_4_ = mx.symbol.Activation(data=batchnorm82_1_4_,
            act_type='relu',
            name="relu82_1_4_")

        conv83_1_4_ = mx.symbol.Convolution(data=relu82_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_4_")
        # conv83_1_4_, output shape: {[2048,7,7]}

        batchnorm83_1_4_ = mx.symbol.BatchNorm(data=conv83_1_4_,
            fix_gamma=True,
            name="batchnorm83_1_4_")
        conv81_1_5_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_5_")
        # conv81_1_5_, output shape: {[32,14,14]}

        batchnorm81_1_5_ = mx.symbol.BatchNorm(data=conv81_1_5_,
            fix_gamma=True,
            name="batchnorm81_1_5_")
        relu81_1_5_ = mx.symbol.Activation(data=batchnorm81_1_5_,
            act_type='relu',
            name="relu81_1_5_")

        conv82_1_5_ = mx.symbol.pad(data=relu81_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_5_ = mx.symbol.Convolution(data=conv82_1_5_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_5_")
        # conv82_1_5_, output shape: {[32,7,7]}

        batchnorm82_1_5_ = mx.symbol.BatchNorm(data=conv82_1_5_,
            fix_gamma=True,
            name="batchnorm82_1_5_")
        relu82_1_5_ = mx.symbol.Activation(data=batchnorm82_1_5_,
            act_type='relu',
            name="relu82_1_5_")

        conv83_1_5_ = mx.symbol.Convolution(data=relu82_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_5_")
        # conv83_1_5_, output shape: {[2048,7,7]}

        batchnorm83_1_5_ = mx.symbol.BatchNorm(data=conv83_1_5_,
            fix_gamma=True,
            name="batchnorm83_1_5_")
        conv81_1_6_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_6_")
        # conv81_1_6_, output shape: {[32,14,14]}

        batchnorm81_1_6_ = mx.symbol.BatchNorm(data=conv81_1_6_,
            fix_gamma=True,
            name="batchnorm81_1_6_")
        relu81_1_6_ = mx.symbol.Activation(data=batchnorm81_1_6_,
            act_type='relu',
            name="relu81_1_6_")

        conv82_1_6_ = mx.symbol.pad(data=relu81_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_6_ = mx.symbol.Convolution(data=conv82_1_6_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_6_")
        # conv82_1_6_, output shape: {[32,7,7]}

        batchnorm82_1_6_ = mx.symbol.BatchNorm(data=conv82_1_6_,
            fix_gamma=True,
            name="batchnorm82_1_6_")
        relu82_1_6_ = mx.symbol.Activation(data=batchnorm82_1_6_,
            act_type='relu',
            name="relu82_1_6_")

        conv83_1_6_ = mx.symbol.Convolution(data=relu82_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_6_")
        # conv83_1_6_, output shape: {[2048,7,7]}

        batchnorm83_1_6_ = mx.symbol.BatchNorm(data=conv83_1_6_,
            fix_gamma=True,
            name="batchnorm83_1_6_")
        conv81_1_7_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_7_")
        # conv81_1_7_, output shape: {[32,14,14]}

        batchnorm81_1_7_ = mx.symbol.BatchNorm(data=conv81_1_7_,
            fix_gamma=True,
            name="batchnorm81_1_7_")
        relu81_1_7_ = mx.symbol.Activation(data=batchnorm81_1_7_,
            act_type='relu',
            name="relu81_1_7_")

        conv82_1_7_ = mx.symbol.pad(data=relu81_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_7_ = mx.symbol.Convolution(data=conv82_1_7_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_7_")
        # conv82_1_7_, output shape: {[32,7,7]}

        batchnorm82_1_7_ = mx.symbol.BatchNorm(data=conv82_1_7_,
            fix_gamma=True,
            name="batchnorm82_1_7_")
        relu82_1_7_ = mx.symbol.Activation(data=batchnorm82_1_7_,
            act_type='relu',
            name="relu82_1_7_")

        conv83_1_7_ = mx.symbol.Convolution(data=relu82_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_7_")
        # conv83_1_7_, output shape: {[2048,7,7]}

        batchnorm83_1_7_ = mx.symbol.BatchNorm(data=conv83_1_7_,
            fix_gamma=True,
            name="batchnorm83_1_7_")
        conv81_1_8_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_8_")
        # conv81_1_8_, output shape: {[32,14,14]}

        batchnorm81_1_8_ = mx.symbol.BatchNorm(data=conv81_1_8_,
            fix_gamma=True,
            name="batchnorm81_1_8_")
        relu81_1_8_ = mx.symbol.Activation(data=batchnorm81_1_8_,
            act_type='relu',
            name="relu81_1_8_")

        conv82_1_8_ = mx.symbol.pad(data=relu81_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_8_ = mx.symbol.Convolution(data=conv82_1_8_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_8_")
        # conv82_1_8_, output shape: {[32,7,7]}

        batchnorm82_1_8_ = mx.symbol.BatchNorm(data=conv82_1_8_,
            fix_gamma=True,
            name="batchnorm82_1_8_")
        relu82_1_8_ = mx.symbol.Activation(data=batchnorm82_1_8_,
            act_type='relu',
            name="relu82_1_8_")

        conv83_1_8_ = mx.symbol.Convolution(data=relu82_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_8_")
        # conv83_1_8_, output shape: {[2048,7,7]}

        batchnorm83_1_8_ = mx.symbol.BatchNorm(data=conv83_1_8_,
            fix_gamma=True,
            name="batchnorm83_1_8_")
        conv81_1_9_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_9_")
        # conv81_1_9_, output shape: {[32,14,14]}

        batchnorm81_1_9_ = mx.symbol.BatchNorm(data=conv81_1_9_,
            fix_gamma=True,
            name="batchnorm81_1_9_")
        relu81_1_9_ = mx.symbol.Activation(data=batchnorm81_1_9_,
            act_type='relu',
            name="relu81_1_9_")

        conv82_1_9_ = mx.symbol.pad(data=relu81_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_9_ = mx.symbol.Convolution(data=conv82_1_9_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_9_")
        # conv82_1_9_, output shape: {[32,7,7]}

        batchnorm82_1_9_ = mx.symbol.BatchNorm(data=conv82_1_9_,
            fix_gamma=True,
            name="batchnorm82_1_9_")
        relu82_1_9_ = mx.symbol.Activation(data=batchnorm82_1_9_,
            act_type='relu',
            name="relu82_1_9_")

        conv83_1_9_ = mx.symbol.Convolution(data=relu82_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_9_")
        # conv83_1_9_, output shape: {[2048,7,7]}

        batchnorm83_1_9_ = mx.symbol.BatchNorm(data=conv83_1_9_,
            fix_gamma=True,
            name="batchnorm83_1_9_")
        conv81_1_10_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_10_")
        # conv81_1_10_, output shape: {[32,14,14]}

        batchnorm81_1_10_ = mx.symbol.BatchNorm(data=conv81_1_10_,
            fix_gamma=True,
            name="batchnorm81_1_10_")
        relu81_1_10_ = mx.symbol.Activation(data=batchnorm81_1_10_,
            act_type='relu',
            name="relu81_1_10_")

        conv82_1_10_ = mx.symbol.pad(data=relu81_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_10_ = mx.symbol.Convolution(data=conv82_1_10_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_10_")
        # conv82_1_10_, output shape: {[32,7,7]}

        batchnorm82_1_10_ = mx.symbol.BatchNorm(data=conv82_1_10_,
            fix_gamma=True,
            name="batchnorm82_1_10_")
        relu82_1_10_ = mx.symbol.Activation(data=batchnorm82_1_10_,
            act_type='relu',
            name="relu82_1_10_")

        conv83_1_10_ = mx.symbol.Convolution(data=relu82_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_10_")
        # conv83_1_10_, output shape: {[2048,7,7]}

        batchnorm83_1_10_ = mx.symbol.BatchNorm(data=conv83_1_10_,
            fix_gamma=True,
            name="batchnorm83_1_10_")
        conv81_1_11_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_11_")
        # conv81_1_11_, output shape: {[32,14,14]}

        batchnorm81_1_11_ = mx.symbol.BatchNorm(data=conv81_1_11_,
            fix_gamma=True,
            name="batchnorm81_1_11_")
        relu81_1_11_ = mx.symbol.Activation(data=batchnorm81_1_11_,
            act_type='relu',
            name="relu81_1_11_")

        conv82_1_11_ = mx.symbol.pad(data=relu81_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_11_ = mx.symbol.Convolution(data=conv82_1_11_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_11_")
        # conv82_1_11_, output shape: {[32,7,7]}

        batchnorm82_1_11_ = mx.symbol.BatchNorm(data=conv82_1_11_,
            fix_gamma=True,
            name="batchnorm82_1_11_")
        relu82_1_11_ = mx.symbol.Activation(data=batchnorm82_1_11_,
            act_type='relu',
            name="relu82_1_11_")

        conv83_1_11_ = mx.symbol.Convolution(data=relu82_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_11_")
        # conv83_1_11_, output shape: {[2048,7,7]}

        batchnorm83_1_11_ = mx.symbol.BatchNorm(data=conv83_1_11_,
            fix_gamma=True,
            name="batchnorm83_1_11_")
        conv81_1_12_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_12_")
        # conv81_1_12_, output shape: {[32,14,14]}

        batchnorm81_1_12_ = mx.symbol.BatchNorm(data=conv81_1_12_,
            fix_gamma=True,
            name="batchnorm81_1_12_")
        relu81_1_12_ = mx.symbol.Activation(data=batchnorm81_1_12_,
            act_type='relu',
            name="relu81_1_12_")

        conv82_1_12_ = mx.symbol.pad(data=relu81_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_12_ = mx.symbol.Convolution(data=conv82_1_12_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_12_")
        # conv82_1_12_, output shape: {[32,7,7]}

        batchnorm82_1_12_ = mx.symbol.BatchNorm(data=conv82_1_12_,
            fix_gamma=True,
            name="batchnorm82_1_12_")
        relu82_1_12_ = mx.symbol.Activation(data=batchnorm82_1_12_,
            act_type='relu',
            name="relu82_1_12_")

        conv83_1_12_ = mx.symbol.Convolution(data=relu82_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_12_")
        # conv83_1_12_, output shape: {[2048,7,7]}

        batchnorm83_1_12_ = mx.symbol.BatchNorm(data=conv83_1_12_,
            fix_gamma=True,
            name="batchnorm83_1_12_")
        conv81_1_13_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_13_")
        # conv81_1_13_, output shape: {[32,14,14]}

        batchnorm81_1_13_ = mx.symbol.BatchNorm(data=conv81_1_13_,
            fix_gamma=True,
            name="batchnorm81_1_13_")
        relu81_1_13_ = mx.symbol.Activation(data=batchnorm81_1_13_,
            act_type='relu',
            name="relu81_1_13_")

        conv82_1_13_ = mx.symbol.pad(data=relu81_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_13_ = mx.symbol.Convolution(data=conv82_1_13_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_13_")
        # conv82_1_13_, output shape: {[32,7,7]}

        batchnorm82_1_13_ = mx.symbol.BatchNorm(data=conv82_1_13_,
            fix_gamma=True,
            name="batchnorm82_1_13_")
        relu82_1_13_ = mx.symbol.Activation(data=batchnorm82_1_13_,
            act_type='relu',
            name="relu82_1_13_")

        conv83_1_13_ = mx.symbol.Convolution(data=relu82_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_13_")
        # conv83_1_13_, output shape: {[2048,7,7]}

        batchnorm83_1_13_ = mx.symbol.BatchNorm(data=conv83_1_13_,
            fix_gamma=True,
            name="batchnorm83_1_13_")
        conv81_1_14_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_14_")
        # conv81_1_14_, output shape: {[32,14,14]}

        batchnorm81_1_14_ = mx.symbol.BatchNorm(data=conv81_1_14_,
            fix_gamma=True,
            name="batchnorm81_1_14_")
        relu81_1_14_ = mx.symbol.Activation(data=batchnorm81_1_14_,
            act_type='relu',
            name="relu81_1_14_")

        conv82_1_14_ = mx.symbol.pad(data=relu81_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_14_ = mx.symbol.Convolution(data=conv82_1_14_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_14_")
        # conv82_1_14_, output shape: {[32,7,7]}

        batchnorm82_1_14_ = mx.symbol.BatchNorm(data=conv82_1_14_,
            fix_gamma=True,
            name="batchnorm82_1_14_")
        relu82_1_14_ = mx.symbol.Activation(data=batchnorm82_1_14_,
            act_type='relu',
            name="relu82_1_14_")

        conv83_1_14_ = mx.symbol.Convolution(data=relu82_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_14_")
        # conv83_1_14_, output shape: {[2048,7,7]}

        batchnorm83_1_14_ = mx.symbol.BatchNorm(data=conv83_1_14_,
            fix_gamma=True,
            name="batchnorm83_1_14_")
        conv81_1_15_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_15_")
        # conv81_1_15_, output shape: {[32,14,14]}

        batchnorm81_1_15_ = mx.symbol.BatchNorm(data=conv81_1_15_,
            fix_gamma=True,
            name="batchnorm81_1_15_")
        relu81_1_15_ = mx.symbol.Activation(data=batchnorm81_1_15_,
            act_type='relu',
            name="relu81_1_15_")

        conv82_1_15_ = mx.symbol.pad(data=relu81_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_15_ = mx.symbol.Convolution(data=conv82_1_15_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_15_")
        # conv82_1_15_, output shape: {[32,7,7]}

        batchnorm82_1_15_ = mx.symbol.BatchNorm(data=conv82_1_15_,
            fix_gamma=True,
            name="batchnorm82_1_15_")
        relu82_1_15_ = mx.symbol.Activation(data=batchnorm82_1_15_,
            act_type='relu',
            name="relu82_1_15_")

        conv83_1_15_ = mx.symbol.Convolution(data=relu82_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_15_")
        # conv83_1_15_, output shape: {[2048,7,7]}

        batchnorm83_1_15_ = mx.symbol.BatchNorm(data=conv83_1_15_,
            fix_gamma=True,
            name="batchnorm83_1_15_")
        conv81_1_16_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_16_")
        # conv81_1_16_, output shape: {[32,14,14]}

        batchnorm81_1_16_ = mx.symbol.BatchNorm(data=conv81_1_16_,
            fix_gamma=True,
            name="batchnorm81_1_16_")
        relu81_1_16_ = mx.symbol.Activation(data=batchnorm81_1_16_,
            act_type='relu',
            name="relu81_1_16_")

        conv82_1_16_ = mx.symbol.pad(data=relu81_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_16_ = mx.symbol.Convolution(data=conv82_1_16_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_16_")
        # conv82_1_16_, output shape: {[32,7,7]}

        batchnorm82_1_16_ = mx.symbol.BatchNorm(data=conv82_1_16_,
            fix_gamma=True,
            name="batchnorm82_1_16_")
        relu82_1_16_ = mx.symbol.Activation(data=batchnorm82_1_16_,
            act_type='relu',
            name="relu82_1_16_")

        conv83_1_16_ = mx.symbol.Convolution(data=relu82_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_16_")
        # conv83_1_16_, output shape: {[2048,7,7]}

        batchnorm83_1_16_ = mx.symbol.BatchNorm(data=conv83_1_16_,
            fix_gamma=True,
            name="batchnorm83_1_16_")
        conv81_1_17_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_17_")
        # conv81_1_17_, output shape: {[32,14,14]}

        batchnorm81_1_17_ = mx.symbol.BatchNorm(data=conv81_1_17_,
            fix_gamma=True,
            name="batchnorm81_1_17_")
        relu81_1_17_ = mx.symbol.Activation(data=batchnorm81_1_17_,
            act_type='relu',
            name="relu81_1_17_")

        conv82_1_17_ = mx.symbol.pad(data=relu81_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_17_ = mx.symbol.Convolution(data=conv82_1_17_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_17_")
        # conv82_1_17_, output shape: {[32,7,7]}

        batchnorm82_1_17_ = mx.symbol.BatchNorm(data=conv82_1_17_,
            fix_gamma=True,
            name="batchnorm82_1_17_")
        relu82_1_17_ = mx.symbol.Activation(data=batchnorm82_1_17_,
            act_type='relu',
            name="relu82_1_17_")

        conv83_1_17_ = mx.symbol.Convolution(data=relu82_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_17_")
        # conv83_1_17_, output shape: {[2048,7,7]}

        batchnorm83_1_17_ = mx.symbol.BatchNorm(data=conv83_1_17_,
            fix_gamma=True,
            name="batchnorm83_1_17_")
        conv81_1_18_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_18_")
        # conv81_1_18_, output shape: {[32,14,14]}

        batchnorm81_1_18_ = mx.symbol.BatchNorm(data=conv81_1_18_,
            fix_gamma=True,
            name="batchnorm81_1_18_")
        relu81_1_18_ = mx.symbol.Activation(data=batchnorm81_1_18_,
            act_type='relu',
            name="relu81_1_18_")

        conv82_1_18_ = mx.symbol.pad(data=relu81_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_18_ = mx.symbol.Convolution(data=conv82_1_18_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_18_")
        # conv82_1_18_, output shape: {[32,7,7]}

        batchnorm82_1_18_ = mx.symbol.BatchNorm(data=conv82_1_18_,
            fix_gamma=True,
            name="batchnorm82_1_18_")
        relu82_1_18_ = mx.symbol.Activation(data=batchnorm82_1_18_,
            act_type='relu',
            name="relu82_1_18_")

        conv83_1_18_ = mx.symbol.Convolution(data=relu82_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_18_")
        # conv83_1_18_, output shape: {[2048,7,7]}

        batchnorm83_1_18_ = mx.symbol.BatchNorm(data=conv83_1_18_,
            fix_gamma=True,
            name="batchnorm83_1_18_")
        conv81_1_19_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_19_")
        # conv81_1_19_, output shape: {[32,14,14]}

        batchnorm81_1_19_ = mx.symbol.BatchNorm(data=conv81_1_19_,
            fix_gamma=True,
            name="batchnorm81_1_19_")
        relu81_1_19_ = mx.symbol.Activation(data=batchnorm81_1_19_,
            act_type='relu',
            name="relu81_1_19_")

        conv82_1_19_ = mx.symbol.pad(data=relu81_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_19_ = mx.symbol.Convolution(data=conv82_1_19_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_19_")
        # conv82_1_19_, output shape: {[32,7,7]}

        batchnorm82_1_19_ = mx.symbol.BatchNorm(data=conv82_1_19_,
            fix_gamma=True,
            name="batchnorm82_1_19_")
        relu82_1_19_ = mx.symbol.Activation(data=batchnorm82_1_19_,
            act_type='relu',
            name="relu82_1_19_")

        conv83_1_19_ = mx.symbol.Convolution(data=relu82_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_19_")
        # conv83_1_19_, output shape: {[2048,7,7]}

        batchnorm83_1_19_ = mx.symbol.BatchNorm(data=conv83_1_19_,
            fix_gamma=True,
            name="batchnorm83_1_19_")
        conv81_1_20_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_20_")
        # conv81_1_20_, output shape: {[32,14,14]}

        batchnorm81_1_20_ = mx.symbol.BatchNorm(data=conv81_1_20_,
            fix_gamma=True,
            name="batchnorm81_1_20_")
        relu81_1_20_ = mx.symbol.Activation(data=batchnorm81_1_20_,
            act_type='relu',
            name="relu81_1_20_")

        conv82_1_20_ = mx.symbol.pad(data=relu81_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_20_ = mx.symbol.Convolution(data=conv82_1_20_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_20_")
        # conv82_1_20_, output shape: {[32,7,7]}

        batchnorm82_1_20_ = mx.symbol.BatchNorm(data=conv82_1_20_,
            fix_gamma=True,
            name="batchnorm82_1_20_")
        relu82_1_20_ = mx.symbol.Activation(data=batchnorm82_1_20_,
            act_type='relu',
            name="relu82_1_20_")

        conv83_1_20_ = mx.symbol.Convolution(data=relu82_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_20_")
        # conv83_1_20_, output shape: {[2048,7,7]}

        batchnorm83_1_20_ = mx.symbol.BatchNorm(data=conv83_1_20_,
            fix_gamma=True,
            name="batchnorm83_1_20_")
        conv81_1_21_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_21_")
        # conv81_1_21_, output shape: {[32,14,14]}

        batchnorm81_1_21_ = mx.symbol.BatchNorm(data=conv81_1_21_,
            fix_gamma=True,
            name="batchnorm81_1_21_")
        relu81_1_21_ = mx.symbol.Activation(data=batchnorm81_1_21_,
            act_type='relu',
            name="relu81_1_21_")

        conv82_1_21_ = mx.symbol.pad(data=relu81_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_21_ = mx.symbol.Convolution(data=conv82_1_21_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_21_")
        # conv82_1_21_, output shape: {[32,7,7]}

        batchnorm82_1_21_ = mx.symbol.BatchNorm(data=conv82_1_21_,
            fix_gamma=True,
            name="batchnorm82_1_21_")
        relu82_1_21_ = mx.symbol.Activation(data=batchnorm82_1_21_,
            act_type='relu',
            name="relu82_1_21_")

        conv83_1_21_ = mx.symbol.Convolution(data=relu82_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_21_")
        # conv83_1_21_, output shape: {[2048,7,7]}

        batchnorm83_1_21_ = mx.symbol.BatchNorm(data=conv83_1_21_,
            fix_gamma=True,
            name="batchnorm83_1_21_")
        conv81_1_22_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_22_")
        # conv81_1_22_, output shape: {[32,14,14]}

        batchnorm81_1_22_ = mx.symbol.BatchNorm(data=conv81_1_22_,
            fix_gamma=True,
            name="batchnorm81_1_22_")
        relu81_1_22_ = mx.symbol.Activation(data=batchnorm81_1_22_,
            act_type='relu',
            name="relu81_1_22_")

        conv82_1_22_ = mx.symbol.pad(data=relu81_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_22_ = mx.symbol.Convolution(data=conv82_1_22_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_22_")
        # conv82_1_22_, output shape: {[32,7,7]}

        batchnorm82_1_22_ = mx.symbol.BatchNorm(data=conv82_1_22_,
            fix_gamma=True,
            name="batchnorm82_1_22_")
        relu82_1_22_ = mx.symbol.Activation(data=batchnorm82_1_22_,
            act_type='relu',
            name="relu82_1_22_")

        conv83_1_22_ = mx.symbol.Convolution(data=relu82_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_22_")
        # conv83_1_22_, output shape: {[2048,7,7]}

        batchnorm83_1_22_ = mx.symbol.BatchNorm(data=conv83_1_22_,
            fix_gamma=True,
            name="batchnorm83_1_22_")
        conv81_1_23_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_23_")
        # conv81_1_23_, output shape: {[32,14,14]}

        batchnorm81_1_23_ = mx.symbol.BatchNorm(data=conv81_1_23_,
            fix_gamma=True,
            name="batchnorm81_1_23_")
        relu81_1_23_ = mx.symbol.Activation(data=batchnorm81_1_23_,
            act_type='relu',
            name="relu81_1_23_")

        conv82_1_23_ = mx.symbol.pad(data=relu81_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_23_ = mx.symbol.Convolution(data=conv82_1_23_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_23_")
        # conv82_1_23_, output shape: {[32,7,7]}

        batchnorm82_1_23_ = mx.symbol.BatchNorm(data=conv82_1_23_,
            fix_gamma=True,
            name="batchnorm82_1_23_")
        relu82_1_23_ = mx.symbol.Activation(data=batchnorm82_1_23_,
            act_type='relu',
            name="relu82_1_23_")

        conv83_1_23_ = mx.symbol.Convolution(data=relu82_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_23_")
        # conv83_1_23_, output shape: {[2048,7,7]}

        batchnorm83_1_23_ = mx.symbol.BatchNorm(data=conv83_1_23_,
            fix_gamma=True,
            name="batchnorm83_1_23_")
        conv81_1_24_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_24_")
        # conv81_1_24_, output shape: {[32,14,14]}

        batchnorm81_1_24_ = mx.symbol.BatchNorm(data=conv81_1_24_,
            fix_gamma=True,
            name="batchnorm81_1_24_")
        relu81_1_24_ = mx.symbol.Activation(data=batchnorm81_1_24_,
            act_type='relu',
            name="relu81_1_24_")

        conv82_1_24_ = mx.symbol.pad(data=relu81_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_24_ = mx.symbol.Convolution(data=conv82_1_24_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_24_")
        # conv82_1_24_, output shape: {[32,7,7]}

        batchnorm82_1_24_ = mx.symbol.BatchNorm(data=conv82_1_24_,
            fix_gamma=True,
            name="batchnorm82_1_24_")
        relu82_1_24_ = mx.symbol.Activation(data=batchnorm82_1_24_,
            act_type='relu',
            name="relu82_1_24_")

        conv83_1_24_ = mx.symbol.Convolution(data=relu82_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_24_")
        # conv83_1_24_, output shape: {[2048,7,7]}

        batchnorm83_1_24_ = mx.symbol.BatchNorm(data=conv83_1_24_,
            fix_gamma=True,
            name="batchnorm83_1_24_")
        conv81_1_25_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_25_")
        # conv81_1_25_, output shape: {[32,14,14]}

        batchnorm81_1_25_ = mx.symbol.BatchNorm(data=conv81_1_25_,
            fix_gamma=True,
            name="batchnorm81_1_25_")
        relu81_1_25_ = mx.symbol.Activation(data=batchnorm81_1_25_,
            act_type='relu',
            name="relu81_1_25_")

        conv82_1_25_ = mx.symbol.pad(data=relu81_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_25_ = mx.symbol.Convolution(data=conv82_1_25_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_25_")
        # conv82_1_25_, output shape: {[32,7,7]}

        batchnorm82_1_25_ = mx.symbol.BatchNorm(data=conv82_1_25_,
            fix_gamma=True,
            name="batchnorm82_1_25_")
        relu82_1_25_ = mx.symbol.Activation(data=batchnorm82_1_25_,
            act_type='relu',
            name="relu82_1_25_")

        conv83_1_25_ = mx.symbol.Convolution(data=relu82_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_25_")
        # conv83_1_25_, output shape: {[2048,7,7]}

        batchnorm83_1_25_ = mx.symbol.BatchNorm(data=conv83_1_25_,
            fix_gamma=True,
            name="batchnorm83_1_25_")
        conv81_1_26_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_26_")
        # conv81_1_26_, output shape: {[32,14,14]}

        batchnorm81_1_26_ = mx.symbol.BatchNorm(data=conv81_1_26_,
            fix_gamma=True,
            name="batchnorm81_1_26_")
        relu81_1_26_ = mx.symbol.Activation(data=batchnorm81_1_26_,
            act_type='relu',
            name="relu81_1_26_")

        conv82_1_26_ = mx.symbol.pad(data=relu81_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_26_ = mx.symbol.Convolution(data=conv82_1_26_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_26_")
        # conv82_1_26_, output shape: {[32,7,7]}

        batchnorm82_1_26_ = mx.symbol.BatchNorm(data=conv82_1_26_,
            fix_gamma=True,
            name="batchnorm82_1_26_")
        relu82_1_26_ = mx.symbol.Activation(data=batchnorm82_1_26_,
            act_type='relu',
            name="relu82_1_26_")

        conv83_1_26_ = mx.symbol.Convolution(data=relu82_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_26_")
        # conv83_1_26_, output shape: {[2048,7,7]}

        batchnorm83_1_26_ = mx.symbol.BatchNorm(data=conv83_1_26_,
            fix_gamma=True,
            name="batchnorm83_1_26_")
        conv81_1_27_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_27_")
        # conv81_1_27_, output shape: {[32,14,14]}

        batchnorm81_1_27_ = mx.symbol.BatchNorm(data=conv81_1_27_,
            fix_gamma=True,
            name="batchnorm81_1_27_")
        relu81_1_27_ = mx.symbol.Activation(data=batchnorm81_1_27_,
            act_type='relu',
            name="relu81_1_27_")

        conv82_1_27_ = mx.symbol.pad(data=relu81_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_27_ = mx.symbol.Convolution(data=conv82_1_27_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_27_")
        # conv82_1_27_, output shape: {[32,7,7]}

        batchnorm82_1_27_ = mx.symbol.BatchNorm(data=conv82_1_27_,
            fix_gamma=True,
            name="batchnorm82_1_27_")
        relu82_1_27_ = mx.symbol.Activation(data=batchnorm82_1_27_,
            act_type='relu',
            name="relu82_1_27_")

        conv83_1_27_ = mx.symbol.Convolution(data=relu82_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_27_")
        # conv83_1_27_, output shape: {[2048,7,7]}

        batchnorm83_1_27_ = mx.symbol.BatchNorm(data=conv83_1_27_,
            fix_gamma=True,
            name="batchnorm83_1_27_")
        conv81_1_28_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_28_")
        # conv81_1_28_, output shape: {[32,14,14]}

        batchnorm81_1_28_ = mx.symbol.BatchNorm(data=conv81_1_28_,
            fix_gamma=True,
            name="batchnorm81_1_28_")
        relu81_1_28_ = mx.symbol.Activation(data=batchnorm81_1_28_,
            act_type='relu',
            name="relu81_1_28_")

        conv82_1_28_ = mx.symbol.pad(data=relu81_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_28_ = mx.symbol.Convolution(data=conv82_1_28_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_28_")
        # conv82_1_28_, output shape: {[32,7,7]}

        batchnorm82_1_28_ = mx.symbol.BatchNorm(data=conv82_1_28_,
            fix_gamma=True,
            name="batchnorm82_1_28_")
        relu82_1_28_ = mx.symbol.Activation(data=batchnorm82_1_28_,
            act_type='relu',
            name="relu82_1_28_")

        conv83_1_28_ = mx.symbol.Convolution(data=relu82_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_28_")
        # conv83_1_28_, output shape: {[2048,7,7]}

        batchnorm83_1_28_ = mx.symbol.BatchNorm(data=conv83_1_28_,
            fix_gamma=True,
            name="batchnorm83_1_28_")
        conv81_1_29_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_29_")
        # conv81_1_29_, output shape: {[32,14,14]}

        batchnorm81_1_29_ = mx.symbol.BatchNorm(data=conv81_1_29_,
            fix_gamma=True,
            name="batchnorm81_1_29_")
        relu81_1_29_ = mx.symbol.Activation(data=batchnorm81_1_29_,
            act_type='relu',
            name="relu81_1_29_")

        conv82_1_29_ = mx.symbol.pad(data=relu81_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_29_ = mx.symbol.Convolution(data=conv82_1_29_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_29_")
        # conv82_1_29_, output shape: {[32,7,7]}

        batchnorm82_1_29_ = mx.symbol.BatchNorm(data=conv82_1_29_,
            fix_gamma=True,
            name="batchnorm82_1_29_")
        relu82_1_29_ = mx.symbol.Activation(data=batchnorm82_1_29_,
            act_type='relu',
            name="relu82_1_29_")

        conv83_1_29_ = mx.symbol.Convolution(data=relu82_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_29_")
        # conv83_1_29_, output shape: {[2048,7,7]}

        batchnorm83_1_29_ = mx.symbol.BatchNorm(data=conv83_1_29_,
            fix_gamma=True,
            name="batchnorm83_1_29_")
        conv81_1_30_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_30_")
        # conv81_1_30_, output shape: {[32,14,14]}

        batchnorm81_1_30_ = mx.symbol.BatchNorm(data=conv81_1_30_,
            fix_gamma=True,
            name="batchnorm81_1_30_")
        relu81_1_30_ = mx.symbol.Activation(data=batchnorm81_1_30_,
            act_type='relu',
            name="relu81_1_30_")

        conv82_1_30_ = mx.symbol.pad(data=relu81_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_30_ = mx.symbol.Convolution(data=conv82_1_30_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_30_")
        # conv82_1_30_, output shape: {[32,7,7]}

        batchnorm82_1_30_ = mx.symbol.BatchNorm(data=conv82_1_30_,
            fix_gamma=True,
            name="batchnorm82_1_30_")
        relu82_1_30_ = mx.symbol.Activation(data=batchnorm82_1_30_,
            act_type='relu',
            name="relu82_1_30_")

        conv83_1_30_ = mx.symbol.Convolution(data=relu82_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_30_")
        # conv83_1_30_, output shape: {[2048,7,7]}

        batchnorm83_1_30_ = mx.symbol.BatchNorm(data=conv83_1_30_,
            fix_gamma=True,
            name="batchnorm83_1_30_")
        conv81_1_31_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_31_")
        # conv81_1_31_, output shape: {[32,14,14]}

        batchnorm81_1_31_ = mx.symbol.BatchNorm(data=conv81_1_31_,
            fix_gamma=True,
            name="batchnorm81_1_31_")
        relu81_1_31_ = mx.symbol.Activation(data=batchnorm81_1_31_,
            act_type='relu',
            name="relu81_1_31_")

        conv82_1_31_ = mx.symbol.pad(data=relu81_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_31_ = mx.symbol.Convolution(data=conv82_1_31_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_31_")
        # conv82_1_31_, output shape: {[32,7,7]}

        batchnorm82_1_31_ = mx.symbol.BatchNorm(data=conv82_1_31_,
            fix_gamma=True,
            name="batchnorm82_1_31_")
        relu82_1_31_ = mx.symbol.Activation(data=batchnorm82_1_31_,
            act_type='relu',
            name="relu82_1_31_")

        conv83_1_31_ = mx.symbol.Convolution(data=relu82_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_31_")
        # conv83_1_31_, output shape: {[2048,7,7]}

        batchnorm83_1_31_ = mx.symbol.BatchNorm(data=conv83_1_31_,
            fix_gamma=True,
            name="batchnorm83_1_31_")
        conv81_1_32_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv81_1_32_")
        # conv81_1_32_, output shape: {[32,14,14]}

        batchnorm81_1_32_ = mx.symbol.BatchNorm(data=conv81_1_32_,
            fix_gamma=True,
            name="batchnorm81_1_32_")
        relu81_1_32_ = mx.symbol.Activation(data=batchnorm81_1_32_,
            act_type='relu',
            name="relu81_1_32_")

        conv82_1_32_ = mx.symbol.pad(data=relu81_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,0,1,0),
            constant_value=0)
        conv82_1_32_ = mx.symbol.Convolution(data=conv82_1_32_,
            kernel=(3,3),
            stride=(2,2),
            num_filter=32,
            no_bias=False,
            name="conv82_1_32_")
        # conv82_1_32_, output shape: {[32,7,7]}

        batchnorm82_1_32_ = mx.symbol.BatchNorm(data=conv82_1_32_,
            fix_gamma=True,
            name="batchnorm82_1_32_")
        relu82_1_32_ = mx.symbol.Activation(data=batchnorm82_1_32_,
            act_type='relu',
            name="relu82_1_32_")

        conv83_1_32_ = mx.symbol.Convolution(data=relu82_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv83_1_32_")
        # conv83_1_32_, output shape: {[2048,7,7]}

        batchnorm83_1_32_ = mx.symbol.BatchNorm(data=conv83_1_32_,
            fix_gamma=True,
            name="batchnorm83_1_32_")
        add84_1_ = batchnorm83_1_1_ + batchnorm83_1_2_ + batchnorm83_1_3_ + batchnorm83_1_4_ + batchnorm83_1_5_ + batchnorm83_1_6_ + batchnorm83_1_7_ + batchnorm83_1_8_ + batchnorm83_1_9_ + batchnorm83_1_10_ + batchnorm83_1_11_ + batchnorm83_1_12_ + batchnorm83_1_13_ + batchnorm83_1_14_ + batchnorm83_1_15_ + batchnorm83_1_16_ + batchnorm83_1_17_ + batchnorm83_1_18_ + batchnorm83_1_19_ + batchnorm83_1_20_ + batchnorm83_1_21_ + batchnorm83_1_22_ + batchnorm83_1_23_ + batchnorm83_1_24_ + batchnorm83_1_25_ + batchnorm83_1_26_ + batchnorm83_1_27_ + batchnorm83_1_28_ + batchnorm83_1_29_ + batchnorm83_1_30_ + batchnorm83_1_31_ + batchnorm83_1_32_
        # add84_1_, output shape: {[2048,7,7]}

        conv80_2_ = mx.symbol.Convolution(data=relu79_,
            kernel=(1,1),
            stride=(2,2),
            num_filter=2048,
            no_bias=False,
            name="conv80_2_")
        # conv80_2_, output shape: {[2048,7,7]}

        batchnorm80_2_ = mx.symbol.BatchNorm(data=conv80_2_,
            fix_gamma=True,
            name="batchnorm80_2_")
        add85_ = add84_1_ + batchnorm80_2_
        # add85_, output shape: {[2048,7,7]}

        relu85_ = mx.symbol.Activation(data=add85_,
            act_type='relu',
            name="relu85_")

        conv87_1_1_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_1_")
        # conv87_1_1_, output shape: {[32,7,7]}

        batchnorm87_1_1_ = mx.symbol.BatchNorm(data=conv87_1_1_,
            fix_gamma=True,
            name="batchnorm87_1_1_")
        relu87_1_1_ = mx.symbol.Activation(data=batchnorm87_1_1_,
            act_type='relu',
            name="relu87_1_1_")

        conv88_1_1_ = mx.symbol.pad(data=relu87_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_1_ = mx.symbol.Convolution(data=conv88_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_1_")
        # conv88_1_1_, output shape: {[32,7,7]}

        batchnorm88_1_1_ = mx.symbol.BatchNorm(data=conv88_1_1_,
            fix_gamma=True,
            name="batchnorm88_1_1_")
        relu88_1_1_ = mx.symbol.Activation(data=batchnorm88_1_1_,
            act_type='relu',
            name="relu88_1_1_")

        conv89_1_1_ = mx.symbol.Convolution(data=relu88_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_1_")
        # conv89_1_1_, output shape: {[2048,7,7]}

        batchnorm89_1_1_ = mx.symbol.BatchNorm(data=conv89_1_1_,
            fix_gamma=True,
            name="batchnorm89_1_1_")
        conv87_1_2_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_2_")
        # conv87_1_2_, output shape: {[32,7,7]}

        batchnorm87_1_2_ = mx.symbol.BatchNorm(data=conv87_1_2_,
            fix_gamma=True,
            name="batchnorm87_1_2_")
        relu87_1_2_ = mx.symbol.Activation(data=batchnorm87_1_2_,
            act_type='relu',
            name="relu87_1_2_")

        conv88_1_2_ = mx.symbol.pad(data=relu87_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_2_ = mx.symbol.Convolution(data=conv88_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_2_")
        # conv88_1_2_, output shape: {[32,7,7]}

        batchnorm88_1_2_ = mx.symbol.BatchNorm(data=conv88_1_2_,
            fix_gamma=True,
            name="batchnorm88_1_2_")
        relu88_1_2_ = mx.symbol.Activation(data=batchnorm88_1_2_,
            act_type='relu',
            name="relu88_1_2_")

        conv89_1_2_ = mx.symbol.Convolution(data=relu88_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_2_")
        # conv89_1_2_, output shape: {[2048,7,7]}

        batchnorm89_1_2_ = mx.symbol.BatchNorm(data=conv89_1_2_,
            fix_gamma=True,
            name="batchnorm89_1_2_")
        conv87_1_3_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_3_")
        # conv87_1_3_, output shape: {[32,7,7]}

        batchnorm87_1_3_ = mx.symbol.BatchNorm(data=conv87_1_3_,
            fix_gamma=True,
            name="batchnorm87_1_3_")
        relu87_1_3_ = mx.symbol.Activation(data=batchnorm87_1_3_,
            act_type='relu',
            name="relu87_1_3_")

        conv88_1_3_ = mx.symbol.pad(data=relu87_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_3_ = mx.symbol.Convolution(data=conv88_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_3_")
        # conv88_1_3_, output shape: {[32,7,7]}

        batchnorm88_1_3_ = mx.symbol.BatchNorm(data=conv88_1_3_,
            fix_gamma=True,
            name="batchnorm88_1_3_")
        relu88_1_3_ = mx.symbol.Activation(data=batchnorm88_1_3_,
            act_type='relu',
            name="relu88_1_3_")

        conv89_1_3_ = mx.symbol.Convolution(data=relu88_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_3_")
        # conv89_1_3_, output shape: {[2048,7,7]}

        batchnorm89_1_3_ = mx.symbol.BatchNorm(data=conv89_1_3_,
            fix_gamma=True,
            name="batchnorm89_1_3_")
        conv87_1_4_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_4_")
        # conv87_1_4_, output shape: {[32,7,7]}

        batchnorm87_1_4_ = mx.symbol.BatchNorm(data=conv87_1_4_,
            fix_gamma=True,
            name="batchnorm87_1_4_")
        relu87_1_4_ = mx.symbol.Activation(data=batchnorm87_1_4_,
            act_type='relu',
            name="relu87_1_4_")

        conv88_1_4_ = mx.symbol.pad(data=relu87_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_4_ = mx.symbol.Convolution(data=conv88_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_4_")
        # conv88_1_4_, output shape: {[32,7,7]}

        batchnorm88_1_4_ = mx.symbol.BatchNorm(data=conv88_1_4_,
            fix_gamma=True,
            name="batchnorm88_1_4_")
        relu88_1_4_ = mx.symbol.Activation(data=batchnorm88_1_4_,
            act_type='relu',
            name="relu88_1_4_")

        conv89_1_4_ = mx.symbol.Convolution(data=relu88_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_4_")
        # conv89_1_4_, output shape: {[2048,7,7]}

        batchnorm89_1_4_ = mx.symbol.BatchNorm(data=conv89_1_4_,
            fix_gamma=True,
            name="batchnorm89_1_4_")
        conv87_1_5_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_5_")
        # conv87_1_5_, output shape: {[32,7,7]}

        batchnorm87_1_5_ = mx.symbol.BatchNorm(data=conv87_1_5_,
            fix_gamma=True,
            name="batchnorm87_1_5_")
        relu87_1_5_ = mx.symbol.Activation(data=batchnorm87_1_5_,
            act_type='relu',
            name="relu87_1_5_")

        conv88_1_5_ = mx.symbol.pad(data=relu87_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_5_ = mx.symbol.Convolution(data=conv88_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_5_")
        # conv88_1_5_, output shape: {[32,7,7]}

        batchnorm88_1_5_ = mx.symbol.BatchNorm(data=conv88_1_5_,
            fix_gamma=True,
            name="batchnorm88_1_5_")
        relu88_1_5_ = mx.symbol.Activation(data=batchnorm88_1_5_,
            act_type='relu',
            name="relu88_1_5_")

        conv89_1_5_ = mx.symbol.Convolution(data=relu88_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_5_")
        # conv89_1_5_, output shape: {[2048,7,7]}

        batchnorm89_1_5_ = mx.symbol.BatchNorm(data=conv89_1_5_,
            fix_gamma=True,
            name="batchnorm89_1_5_")
        conv87_1_6_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_6_")
        # conv87_1_6_, output shape: {[32,7,7]}

        batchnorm87_1_6_ = mx.symbol.BatchNorm(data=conv87_1_6_,
            fix_gamma=True,
            name="batchnorm87_1_6_")
        relu87_1_6_ = mx.symbol.Activation(data=batchnorm87_1_6_,
            act_type='relu',
            name="relu87_1_6_")

        conv88_1_6_ = mx.symbol.pad(data=relu87_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_6_ = mx.symbol.Convolution(data=conv88_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_6_")
        # conv88_1_6_, output shape: {[32,7,7]}

        batchnorm88_1_6_ = mx.symbol.BatchNorm(data=conv88_1_6_,
            fix_gamma=True,
            name="batchnorm88_1_6_")
        relu88_1_6_ = mx.symbol.Activation(data=batchnorm88_1_6_,
            act_type='relu',
            name="relu88_1_6_")

        conv89_1_6_ = mx.symbol.Convolution(data=relu88_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_6_")
        # conv89_1_6_, output shape: {[2048,7,7]}

        batchnorm89_1_6_ = mx.symbol.BatchNorm(data=conv89_1_6_,
            fix_gamma=True,
            name="batchnorm89_1_6_")
        conv87_1_7_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_7_")
        # conv87_1_7_, output shape: {[32,7,7]}

        batchnorm87_1_7_ = mx.symbol.BatchNorm(data=conv87_1_7_,
            fix_gamma=True,
            name="batchnorm87_1_7_")
        relu87_1_7_ = mx.symbol.Activation(data=batchnorm87_1_7_,
            act_type='relu',
            name="relu87_1_7_")

        conv88_1_7_ = mx.symbol.pad(data=relu87_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_7_ = mx.symbol.Convolution(data=conv88_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_7_")
        # conv88_1_7_, output shape: {[32,7,7]}

        batchnorm88_1_7_ = mx.symbol.BatchNorm(data=conv88_1_7_,
            fix_gamma=True,
            name="batchnorm88_1_7_")
        relu88_1_7_ = mx.symbol.Activation(data=batchnorm88_1_7_,
            act_type='relu',
            name="relu88_1_7_")

        conv89_1_7_ = mx.symbol.Convolution(data=relu88_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_7_")
        # conv89_1_7_, output shape: {[2048,7,7]}

        batchnorm89_1_7_ = mx.symbol.BatchNorm(data=conv89_1_7_,
            fix_gamma=True,
            name="batchnorm89_1_7_")
        conv87_1_8_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_8_")
        # conv87_1_8_, output shape: {[32,7,7]}

        batchnorm87_1_8_ = mx.symbol.BatchNorm(data=conv87_1_8_,
            fix_gamma=True,
            name="batchnorm87_1_8_")
        relu87_1_8_ = mx.symbol.Activation(data=batchnorm87_1_8_,
            act_type='relu',
            name="relu87_1_8_")

        conv88_1_8_ = mx.symbol.pad(data=relu87_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_8_ = mx.symbol.Convolution(data=conv88_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_8_")
        # conv88_1_8_, output shape: {[32,7,7]}

        batchnorm88_1_8_ = mx.symbol.BatchNorm(data=conv88_1_8_,
            fix_gamma=True,
            name="batchnorm88_1_8_")
        relu88_1_8_ = mx.symbol.Activation(data=batchnorm88_1_8_,
            act_type='relu',
            name="relu88_1_8_")

        conv89_1_8_ = mx.symbol.Convolution(data=relu88_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_8_")
        # conv89_1_8_, output shape: {[2048,7,7]}

        batchnorm89_1_8_ = mx.symbol.BatchNorm(data=conv89_1_8_,
            fix_gamma=True,
            name="batchnorm89_1_8_")
        conv87_1_9_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_9_")
        # conv87_1_9_, output shape: {[32,7,7]}

        batchnorm87_1_9_ = mx.symbol.BatchNorm(data=conv87_1_9_,
            fix_gamma=True,
            name="batchnorm87_1_9_")
        relu87_1_9_ = mx.symbol.Activation(data=batchnorm87_1_9_,
            act_type='relu',
            name="relu87_1_9_")

        conv88_1_9_ = mx.symbol.pad(data=relu87_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_9_ = mx.symbol.Convolution(data=conv88_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_9_")
        # conv88_1_9_, output shape: {[32,7,7]}

        batchnorm88_1_9_ = mx.symbol.BatchNorm(data=conv88_1_9_,
            fix_gamma=True,
            name="batchnorm88_1_9_")
        relu88_1_9_ = mx.symbol.Activation(data=batchnorm88_1_9_,
            act_type='relu',
            name="relu88_1_9_")

        conv89_1_9_ = mx.symbol.Convolution(data=relu88_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_9_")
        # conv89_1_9_, output shape: {[2048,7,7]}

        batchnorm89_1_9_ = mx.symbol.BatchNorm(data=conv89_1_9_,
            fix_gamma=True,
            name="batchnorm89_1_9_")
        conv87_1_10_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_10_")
        # conv87_1_10_, output shape: {[32,7,7]}

        batchnorm87_1_10_ = mx.symbol.BatchNorm(data=conv87_1_10_,
            fix_gamma=True,
            name="batchnorm87_1_10_")
        relu87_1_10_ = mx.symbol.Activation(data=batchnorm87_1_10_,
            act_type='relu',
            name="relu87_1_10_")

        conv88_1_10_ = mx.symbol.pad(data=relu87_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_10_ = mx.symbol.Convolution(data=conv88_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_10_")
        # conv88_1_10_, output shape: {[32,7,7]}

        batchnorm88_1_10_ = mx.symbol.BatchNorm(data=conv88_1_10_,
            fix_gamma=True,
            name="batchnorm88_1_10_")
        relu88_1_10_ = mx.symbol.Activation(data=batchnorm88_1_10_,
            act_type='relu',
            name="relu88_1_10_")

        conv89_1_10_ = mx.symbol.Convolution(data=relu88_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_10_")
        # conv89_1_10_, output shape: {[2048,7,7]}

        batchnorm89_1_10_ = mx.symbol.BatchNorm(data=conv89_1_10_,
            fix_gamma=True,
            name="batchnorm89_1_10_")
        conv87_1_11_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_11_")
        # conv87_1_11_, output shape: {[32,7,7]}

        batchnorm87_1_11_ = mx.symbol.BatchNorm(data=conv87_1_11_,
            fix_gamma=True,
            name="batchnorm87_1_11_")
        relu87_1_11_ = mx.symbol.Activation(data=batchnorm87_1_11_,
            act_type='relu',
            name="relu87_1_11_")

        conv88_1_11_ = mx.symbol.pad(data=relu87_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_11_ = mx.symbol.Convolution(data=conv88_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_11_")
        # conv88_1_11_, output shape: {[32,7,7]}

        batchnorm88_1_11_ = mx.symbol.BatchNorm(data=conv88_1_11_,
            fix_gamma=True,
            name="batchnorm88_1_11_")
        relu88_1_11_ = mx.symbol.Activation(data=batchnorm88_1_11_,
            act_type='relu',
            name="relu88_1_11_")

        conv89_1_11_ = mx.symbol.Convolution(data=relu88_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_11_")
        # conv89_1_11_, output shape: {[2048,7,7]}

        batchnorm89_1_11_ = mx.symbol.BatchNorm(data=conv89_1_11_,
            fix_gamma=True,
            name="batchnorm89_1_11_")
        conv87_1_12_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_12_")
        # conv87_1_12_, output shape: {[32,7,7]}

        batchnorm87_1_12_ = mx.symbol.BatchNorm(data=conv87_1_12_,
            fix_gamma=True,
            name="batchnorm87_1_12_")
        relu87_1_12_ = mx.symbol.Activation(data=batchnorm87_1_12_,
            act_type='relu',
            name="relu87_1_12_")

        conv88_1_12_ = mx.symbol.pad(data=relu87_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_12_ = mx.symbol.Convolution(data=conv88_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_12_")
        # conv88_1_12_, output shape: {[32,7,7]}

        batchnorm88_1_12_ = mx.symbol.BatchNorm(data=conv88_1_12_,
            fix_gamma=True,
            name="batchnorm88_1_12_")
        relu88_1_12_ = mx.symbol.Activation(data=batchnorm88_1_12_,
            act_type='relu',
            name="relu88_1_12_")

        conv89_1_12_ = mx.symbol.Convolution(data=relu88_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_12_")
        # conv89_1_12_, output shape: {[2048,7,7]}

        batchnorm89_1_12_ = mx.symbol.BatchNorm(data=conv89_1_12_,
            fix_gamma=True,
            name="batchnorm89_1_12_")
        conv87_1_13_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_13_")
        # conv87_1_13_, output shape: {[32,7,7]}

        batchnorm87_1_13_ = mx.symbol.BatchNorm(data=conv87_1_13_,
            fix_gamma=True,
            name="batchnorm87_1_13_")
        relu87_1_13_ = mx.symbol.Activation(data=batchnorm87_1_13_,
            act_type='relu',
            name="relu87_1_13_")

        conv88_1_13_ = mx.symbol.pad(data=relu87_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_13_ = mx.symbol.Convolution(data=conv88_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_13_")
        # conv88_1_13_, output shape: {[32,7,7]}

        batchnorm88_1_13_ = mx.symbol.BatchNorm(data=conv88_1_13_,
            fix_gamma=True,
            name="batchnorm88_1_13_")
        relu88_1_13_ = mx.symbol.Activation(data=batchnorm88_1_13_,
            act_type='relu',
            name="relu88_1_13_")

        conv89_1_13_ = mx.symbol.Convolution(data=relu88_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_13_")
        # conv89_1_13_, output shape: {[2048,7,7]}

        batchnorm89_1_13_ = mx.symbol.BatchNorm(data=conv89_1_13_,
            fix_gamma=True,
            name="batchnorm89_1_13_")
        conv87_1_14_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_14_")
        # conv87_1_14_, output shape: {[32,7,7]}

        batchnorm87_1_14_ = mx.symbol.BatchNorm(data=conv87_1_14_,
            fix_gamma=True,
            name="batchnorm87_1_14_")
        relu87_1_14_ = mx.symbol.Activation(data=batchnorm87_1_14_,
            act_type='relu',
            name="relu87_1_14_")

        conv88_1_14_ = mx.symbol.pad(data=relu87_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_14_ = mx.symbol.Convolution(data=conv88_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_14_")
        # conv88_1_14_, output shape: {[32,7,7]}

        batchnorm88_1_14_ = mx.symbol.BatchNorm(data=conv88_1_14_,
            fix_gamma=True,
            name="batchnorm88_1_14_")
        relu88_1_14_ = mx.symbol.Activation(data=batchnorm88_1_14_,
            act_type='relu',
            name="relu88_1_14_")

        conv89_1_14_ = mx.symbol.Convolution(data=relu88_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_14_")
        # conv89_1_14_, output shape: {[2048,7,7]}

        batchnorm89_1_14_ = mx.symbol.BatchNorm(data=conv89_1_14_,
            fix_gamma=True,
            name="batchnorm89_1_14_")
        conv87_1_15_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_15_")
        # conv87_1_15_, output shape: {[32,7,7]}

        batchnorm87_1_15_ = mx.symbol.BatchNorm(data=conv87_1_15_,
            fix_gamma=True,
            name="batchnorm87_1_15_")
        relu87_1_15_ = mx.symbol.Activation(data=batchnorm87_1_15_,
            act_type='relu',
            name="relu87_1_15_")

        conv88_1_15_ = mx.symbol.pad(data=relu87_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_15_ = mx.symbol.Convolution(data=conv88_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_15_")
        # conv88_1_15_, output shape: {[32,7,7]}

        batchnorm88_1_15_ = mx.symbol.BatchNorm(data=conv88_1_15_,
            fix_gamma=True,
            name="batchnorm88_1_15_")
        relu88_1_15_ = mx.symbol.Activation(data=batchnorm88_1_15_,
            act_type='relu',
            name="relu88_1_15_")

        conv89_1_15_ = mx.symbol.Convolution(data=relu88_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_15_")
        # conv89_1_15_, output shape: {[2048,7,7]}

        batchnorm89_1_15_ = mx.symbol.BatchNorm(data=conv89_1_15_,
            fix_gamma=True,
            name="batchnorm89_1_15_")
        conv87_1_16_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_16_")
        # conv87_1_16_, output shape: {[32,7,7]}

        batchnorm87_1_16_ = mx.symbol.BatchNorm(data=conv87_1_16_,
            fix_gamma=True,
            name="batchnorm87_1_16_")
        relu87_1_16_ = mx.symbol.Activation(data=batchnorm87_1_16_,
            act_type='relu',
            name="relu87_1_16_")

        conv88_1_16_ = mx.symbol.pad(data=relu87_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_16_ = mx.symbol.Convolution(data=conv88_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_16_")
        # conv88_1_16_, output shape: {[32,7,7]}

        batchnorm88_1_16_ = mx.symbol.BatchNorm(data=conv88_1_16_,
            fix_gamma=True,
            name="batchnorm88_1_16_")
        relu88_1_16_ = mx.symbol.Activation(data=batchnorm88_1_16_,
            act_type='relu',
            name="relu88_1_16_")

        conv89_1_16_ = mx.symbol.Convolution(data=relu88_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_16_")
        # conv89_1_16_, output shape: {[2048,7,7]}

        batchnorm89_1_16_ = mx.symbol.BatchNorm(data=conv89_1_16_,
            fix_gamma=True,
            name="batchnorm89_1_16_")
        conv87_1_17_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_17_")
        # conv87_1_17_, output shape: {[32,7,7]}

        batchnorm87_1_17_ = mx.symbol.BatchNorm(data=conv87_1_17_,
            fix_gamma=True,
            name="batchnorm87_1_17_")
        relu87_1_17_ = mx.symbol.Activation(data=batchnorm87_1_17_,
            act_type='relu',
            name="relu87_1_17_")

        conv88_1_17_ = mx.symbol.pad(data=relu87_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_17_ = mx.symbol.Convolution(data=conv88_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_17_")
        # conv88_1_17_, output shape: {[32,7,7]}

        batchnorm88_1_17_ = mx.symbol.BatchNorm(data=conv88_1_17_,
            fix_gamma=True,
            name="batchnorm88_1_17_")
        relu88_1_17_ = mx.symbol.Activation(data=batchnorm88_1_17_,
            act_type='relu',
            name="relu88_1_17_")

        conv89_1_17_ = mx.symbol.Convolution(data=relu88_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_17_")
        # conv89_1_17_, output shape: {[2048,7,7]}

        batchnorm89_1_17_ = mx.symbol.BatchNorm(data=conv89_1_17_,
            fix_gamma=True,
            name="batchnorm89_1_17_")
        conv87_1_18_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_18_")
        # conv87_1_18_, output shape: {[32,7,7]}

        batchnorm87_1_18_ = mx.symbol.BatchNorm(data=conv87_1_18_,
            fix_gamma=True,
            name="batchnorm87_1_18_")
        relu87_1_18_ = mx.symbol.Activation(data=batchnorm87_1_18_,
            act_type='relu',
            name="relu87_1_18_")

        conv88_1_18_ = mx.symbol.pad(data=relu87_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_18_ = mx.symbol.Convolution(data=conv88_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_18_")
        # conv88_1_18_, output shape: {[32,7,7]}

        batchnorm88_1_18_ = mx.symbol.BatchNorm(data=conv88_1_18_,
            fix_gamma=True,
            name="batchnorm88_1_18_")
        relu88_1_18_ = mx.symbol.Activation(data=batchnorm88_1_18_,
            act_type='relu',
            name="relu88_1_18_")

        conv89_1_18_ = mx.symbol.Convolution(data=relu88_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_18_")
        # conv89_1_18_, output shape: {[2048,7,7]}

        batchnorm89_1_18_ = mx.symbol.BatchNorm(data=conv89_1_18_,
            fix_gamma=True,
            name="batchnorm89_1_18_")
        conv87_1_19_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_19_")
        # conv87_1_19_, output shape: {[32,7,7]}

        batchnorm87_1_19_ = mx.symbol.BatchNorm(data=conv87_1_19_,
            fix_gamma=True,
            name="batchnorm87_1_19_")
        relu87_1_19_ = mx.symbol.Activation(data=batchnorm87_1_19_,
            act_type='relu',
            name="relu87_1_19_")

        conv88_1_19_ = mx.symbol.pad(data=relu87_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_19_ = mx.symbol.Convolution(data=conv88_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_19_")
        # conv88_1_19_, output shape: {[32,7,7]}

        batchnorm88_1_19_ = mx.symbol.BatchNorm(data=conv88_1_19_,
            fix_gamma=True,
            name="batchnorm88_1_19_")
        relu88_1_19_ = mx.symbol.Activation(data=batchnorm88_1_19_,
            act_type='relu',
            name="relu88_1_19_")

        conv89_1_19_ = mx.symbol.Convolution(data=relu88_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_19_")
        # conv89_1_19_, output shape: {[2048,7,7]}

        batchnorm89_1_19_ = mx.symbol.BatchNorm(data=conv89_1_19_,
            fix_gamma=True,
            name="batchnorm89_1_19_")
        conv87_1_20_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_20_")
        # conv87_1_20_, output shape: {[32,7,7]}

        batchnorm87_1_20_ = mx.symbol.BatchNorm(data=conv87_1_20_,
            fix_gamma=True,
            name="batchnorm87_1_20_")
        relu87_1_20_ = mx.symbol.Activation(data=batchnorm87_1_20_,
            act_type='relu',
            name="relu87_1_20_")

        conv88_1_20_ = mx.symbol.pad(data=relu87_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_20_ = mx.symbol.Convolution(data=conv88_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_20_")
        # conv88_1_20_, output shape: {[32,7,7]}

        batchnorm88_1_20_ = mx.symbol.BatchNorm(data=conv88_1_20_,
            fix_gamma=True,
            name="batchnorm88_1_20_")
        relu88_1_20_ = mx.symbol.Activation(data=batchnorm88_1_20_,
            act_type='relu',
            name="relu88_1_20_")

        conv89_1_20_ = mx.symbol.Convolution(data=relu88_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_20_")
        # conv89_1_20_, output shape: {[2048,7,7]}

        batchnorm89_1_20_ = mx.symbol.BatchNorm(data=conv89_1_20_,
            fix_gamma=True,
            name="batchnorm89_1_20_")
        conv87_1_21_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_21_")
        # conv87_1_21_, output shape: {[32,7,7]}

        batchnorm87_1_21_ = mx.symbol.BatchNorm(data=conv87_1_21_,
            fix_gamma=True,
            name="batchnorm87_1_21_")
        relu87_1_21_ = mx.symbol.Activation(data=batchnorm87_1_21_,
            act_type='relu',
            name="relu87_1_21_")

        conv88_1_21_ = mx.symbol.pad(data=relu87_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_21_ = mx.symbol.Convolution(data=conv88_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_21_")
        # conv88_1_21_, output shape: {[32,7,7]}

        batchnorm88_1_21_ = mx.symbol.BatchNorm(data=conv88_1_21_,
            fix_gamma=True,
            name="batchnorm88_1_21_")
        relu88_1_21_ = mx.symbol.Activation(data=batchnorm88_1_21_,
            act_type='relu',
            name="relu88_1_21_")

        conv89_1_21_ = mx.symbol.Convolution(data=relu88_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_21_")
        # conv89_1_21_, output shape: {[2048,7,7]}

        batchnorm89_1_21_ = mx.symbol.BatchNorm(data=conv89_1_21_,
            fix_gamma=True,
            name="batchnorm89_1_21_")
        conv87_1_22_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_22_")
        # conv87_1_22_, output shape: {[32,7,7]}

        batchnorm87_1_22_ = mx.symbol.BatchNorm(data=conv87_1_22_,
            fix_gamma=True,
            name="batchnorm87_1_22_")
        relu87_1_22_ = mx.symbol.Activation(data=batchnorm87_1_22_,
            act_type='relu',
            name="relu87_1_22_")

        conv88_1_22_ = mx.symbol.pad(data=relu87_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_22_ = mx.symbol.Convolution(data=conv88_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_22_")
        # conv88_1_22_, output shape: {[32,7,7]}

        batchnorm88_1_22_ = mx.symbol.BatchNorm(data=conv88_1_22_,
            fix_gamma=True,
            name="batchnorm88_1_22_")
        relu88_1_22_ = mx.symbol.Activation(data=batchnorm88_1_22_,
            act_type='relu',
            name="relu88_1_22_")

        conv89_1_22_ = mx.symbol.Convolution(data=relu88_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_22_")
        # conv89_1_22_, output shape: {[2048,7,7]}

        batchnorm89_1_22_ = mx.symbol.BatchNorm(data=conv89_1_22_,
            fix_gamma=True,
            name="batchnorm89_1_22_")
        conv87_1_23_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_23_")
        # conv87_1_23_, output shape: {[32,7,7]}

        batchnorm87_1_23_ = mx.symbol.BatchNorm(data=conv87_1_23_,
            fix_gamma=True,
            name="batchnorm87_1_23_")
        relu87_1_23_ = mx.symbol.Activation(data=batchnorm87_1_23_,
            act_type='relu',
            name="relu87_1_23_")

        conv88_1_23_ = mx.symbol.pad(data=relu87_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_23_ = mx.symbol.Convolution(data=conv88_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_23_")
        # conv88_1_23_, output shape: {[32,7,7]}

        batchnorm88_1_23_ = mx.symbol.BatchNorm(data=conv88_1_23_,
            fix_gamma=True,
            name="batchnorm88_1_23_")
        relu88_1_23_ = mx.symbol.Activation(data=batchnorm88_1_23_,
            act_type='relu',
            name="relu88_1_23_")

        conv89_1_23_ = mx.symbol.Convolution(data=relu88_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_23_")
        # conv89_1_23_, output shape: {[2048,7,7]}

        batchnorm89_1_23_ = mx.symbol.BatchNorm(data=conv89_1_23_,
            fix_gamma=True,
            name="batchnorm89_1_23_")
        conv87_1_24_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_24_")
        # conv87_1_24_, output shape: {[32,7,7]}

        batchnorm87_1_24_ = mx.symbol.BatchNorm(data=conv87_1_24_,
            fix_gamma=True,
            name="batchnorm87_1_24_")
        relu87_1_24_ = mx.symbol.Activation(data=batchnorm87_1_24_,
            act_type='relu',
            name="relu87_1_24_")

        conv88_1_24_ = mx.symbol.pad(data=relu87_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_24_ = mx.symbol.Convolution(data=conv88_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_24_")
        # conv88_1_24_, output shape: {[32,7,7]}

        batchnorm88_1_24_ = mx.symbol.BatchNorm(data=conv88_1_24_,
            fix_gamma=True,
            name="batchnorm88_1_24_")
        relu88_1_24_ = mx.symbol.Activation(data=batchnorm88_1_24_,
            act_type='relu',
            name="relu88_1_24_")

        conv89_1_24_ = mx.symbol.Convolution(data=relu88_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_24_")
        # conv89_1_24_, output shape: {[2048,7,7]}

        batchnorm89_1_24_ = mx.symbol.BatchNorm(data=conv89_1_24_,
            fix_gamma=True,
            name="batchnorm89_1_24_")
        conv87_1_25_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_25_")
        # conv87_1_25_, output shape: {[32,7,7]}

        batchnorm87_1_25_ = mx.symbol.BatchNorm(data=conv87_1_25_,
            fix_gamma=True,
            name="batchnorm87_1_25_")
        relu87_1_25_ = mx.symbol.Activation(data=batchnorm87_1_25_,
            act_type='relu',
            name="relu87_1_25_")

        conv88_1_25_ = mx.symbol.pad(data=relu87_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_25_ = mx.symbol.Convolution(data=conv88_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_25_")
        # conv88_1_25_, output shape: {[32,7,7]}

        batchnorm88_1_25_ = mx.symbol.BatchNorm(data=conv88_1_25_,
            fix_gamma=True,
            name="batchnorm88_1_25_")
        relu88_1_25_ = mx.symbol.Activation(data=batchnorm88_1_25_,
            act_type='relu',
            name="relu88_1_25_")

        conv89_1_25_ = mx.symbol.Convolution(data=relu88_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_25_")
        # conv89_1_25_, output shape: {[2048,7,7]}

        batchnorm89_1_25_ = mx.symbol.BatchNorm(data=conv89_1_25_,
            fix_gamma=True,
            name="batchnorm89_1_25_")
        conv87_1_26_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_26_")
        # conv87_1_26_, output shape: {[32,7,7]}

        batchnorm87_1_26_ = mx.symbol.BatchNorm(data=conv87_1_26_,
            fix_gamma=True,
            name="batchnorm87_1_26_")
        relu87_1_26_ = mx.symbol.Activation(data=batchnorm87_1_26_,
            act_type='relu',
            name="relu87_1_26_")

        conv88_1_26_ = mx.symbol.pad(data=relu87_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_26_ = mx.symbol.Convolution(data=conv88_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_26_")
        # conv88_1_26_, output shape: {[32,7,7]}

        batchnorm88_1_26_ = mx.symbol.BatchNorm(data=conv88_1_26_,
            fix_gamma=True,
            name="batchnorm88_1_26_")
        relu88_1_26_ = mx.symbol.Activation(data=batchnorm88_1_26_,
            act_type='relu',
            name="relu88_1_26_")

        conv89_1_26_ = mx.symbol.Convolution(data=relu88_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_26_")
        # conv89_1_26_, output shape: {[2048,7,7]}

        batchnorm89_1_26_ = mx.symbol.BatchNorm(data=conv89_1_26_,
            fix_gamma=True,
            name="batchnorm89_1_26_")
        conv87_1_27_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_27_")
        # conv87_1_27_, output shape: {[32,7,7]}

        batchnorm87_1_27_ = mx.symbol.BatchNorm(data=conv87_1_27_,
            fix_gamma=True,
            name="batchnorm87_1_27_")
        relu87_1_27_ = mx.symbol.Activation(data=batchnorm87_1_27_,
            act_type='relu',
            name="relu87_1_27_")

        conv88_1_27_ = mx.symbol.pad(data=relu87_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_27_ = mx.symbol.Convolution(data=conv88_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_27_")
        # conv88_1_27_, output shape: {[32,7,7]}

        batchnorm88_1_27_ = mx.symbol.BatchNorm(data=conv88_1_27_,
            fix_gamma=True,
            name="batchnorm88_1_27_")
        relu88_1_27_ = mx.symbol.Activation(data=batchnorm88_1_27_,
            act_type='relu',
            name="relu88_1_27_")

        conv89_1_27_ = mx.symbol.Convolution(data=relu88_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_27_")
        # conv89_1_27_, output shape: {[2048,7,7]}

        batchnorm89_1_27_ = mx.symbol.BatchNorm(data=conv89_1_27_,
            fix_gamma=True,
            name="batchnorm89_1_27_")
        conv87_1_28_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_28_")
        # conv87_1_28_, output shape: {[32,7,7]}

        batchnorm87_1_28_ = mx.symbol.BatchNorm(data=conv87_1_28_,
            fix_gamma=True,
            name="batchnorm87_1_28_")
        relu87_1_28_ = mx.symbol.Activation(data=batchnorm87_1_28_,
            act_type='relu',
            name="relu87_1_28_")

        conv88_1_28_ = mx.symbol.pad(data=relu87_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_28_ = mx.symbol.Convolution(data=conv88_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_28_")
        # conv88_1_28_, output shape: {[32,7,7]}

        batchnorm88_1_28_ = mx.symbol.BatchNorm(data=conv88_1_28_,
            fix_gamma=True,
            name="batchnorm88_1_28_")
        relu88_1_28_ = mx.symbol.Activation(data=batchnorm88_1_28_,
            act_type='relu',
            name="relu88_1_28_")

        conv89_1_28_ = mx.symbol.Convolution(data=relu88_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_28_")
        # conv89_1_28_, output shape: {[2048,7,7]}

        batchnorm89_1_28_ = mx.symbol.BatchNorm(data=conv89_1_28_,
            fix_gamma=True,
            name="batchnorm89_1_28_")
        conv87_1_29_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_29_")
        # conv87_1_29_, output shape: {[32,7,7]}

        batchnorm87_1_29_ = mx.symbol.BatchNorm(data=conv87_1_29_,
            fix_gamma=True,
            name="batchnorm87_1_29_")
        relu87_1_29_ = mx.symbol.Activation(data=batchnorm87_1_29_,
            act_type='relu',
            name="relu87_1_29_")

        conv88_1_29_ = mx.symbol.pad(data=relu87_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_29_ = mx.symbol.Convolution(data=conv88_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_29_")
        # conv88_1_29_, output shape: {[32,7,7]}

        batchnorm88_1_29_ = mx.symbol.BatchNorm(data=conv88_1_29_,
            fix_gamma=True,
            name="batchnorm88_1_29_")
        relu88_1_29_ = mx.symbol.Activation(data=batchnorm88_1_29_,
            act_type='relu',
            name="relu88_1_29_")

        conv89_1_29_ = mx.symbol.Convolution(data=relu88_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_29_")
        # conv89_1_29_, output shape: {[2048,7,7]}

        batchnorm89_1_29_ = mx.symbol.BatchNorm(data=conv89_1_29_,
            fix_gamma=True,
            name="batchnorm89_1_29_")
        conv87_1_30_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_30_")
        # conv87_1_30_, output shape: {[32,7,7]}

        batchnorm87_1_30_ = mx.symbol.BatchNorm(data=conv87_1_30_,
            fix_gamma=True,
            name="batchnorm87_1_30_")
        relu87_1_30_ = mx.symbol.Activation(data=batchnorm87_1_30_,
            act_type='relu',
            name="relu87_1_30_")

        conv88_1_30_ = mx.symbol.pad(data=relu87_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_30_ = mx.symbol.Convolution(data=conv88_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_30_")
        # conv88_1_30_, output shape: {[32,7,7]}

        batchnorm88_1_30_ = mx.symbol.BatchNorm(data=conv88_1_30_,
            fix_gamma=True,
            name="batchnorm88_1_30_")
        relu88_1_30_ = mx.symbol.Activation(data=batchnorm88_1_30_,
            act_type='relu',
            name="relu88_1_30_")

        conv89_1_30_ = mx.symbol.Convolution(data=relu88_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_30_")
        # conv89_1_30_, output shape: {[2048,7,7]}

        batchnorm89_1_30_ = mx.symbol.BatchNorm(data=conv89_1_30_,
            fix_gamma=True,
            name="batchnorm89_1_30_")
        conv87_1_31_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_31_")
        # conv87_1_31_, output shape: {[32,7,7]}

        batchnorm87_1_31_ = mx.symbol.BatchNorm(data=conv87_1_31_,
            fix_gamma=True,
            name="batchnorm87_1_31_")
        relu87_1_31_ = mx.symbol.Activation(data=batchnorm87_1_31_,
            act_type='relu',
            name="relu87_1_31_")

        conv88_1_31_ = mx.symbol.pad(data=relu87_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_31_ = mx.symbol.Convolution(data=conv88_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_31_")
        # conv88_1_31_, output shape: {[32,7,7]}

        batchnorm88_1_31_ = mx.symbol.BatchNorm(data=conv88_1_31_,
            fix_gamma=True,
            name="batchnorm88_1_31_")
        relu88_1_31_ = mx.symbol.Activation(data=batchnorm88_1_31_,
            act_type='relu',
            name="relu88_1_31_")

        conv89_1_31_ = mx.symbol.Convolution(data=relu88_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_31_")
        # conv89_1_31_, output shape: {[2048,7,7]}

        batchnorm89_1_31_ = mx.symbol.BatchNorm(data=conv89_1_31_,
            fix_gamma=True,
            name="batchnorm89_1_31_")
        conv87_1_32_ = mx.symbol.Convolution(data=relu85_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv87_1_32_")
        # conv87_1_32_, output shape: {[32,7,7]}

        batchnorm87_1_32_ = mx.symbol.BatchNorm(data=conv87_1_32_,
            fix_gamma=True,
            name="batchnorm87_1_32_")
        relu87_1_32_ = mx.symbol.Activation(data=batchnorm87_1_32_,
            act_type='relu',
            name="relu87_1_32_")

        conv88_1_32_ = mx.symbol.pad(data=relu87_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv88_1_32_ = mx.symbol.Convolution(data=conv88_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv88_1_32_")
        # conv88_1_32_, output shape: {[32,7,7]}

        batchnorm88_1_32_ = mx.symbol.BatchNorm(data=conv88_1_32_,
            fix_gamma=True,
            name="batchnorm88_1_32_")
        relu88_1_32_ = mx.symbol.Activation(data=batchnorm88_1_32_,
            act_type='relu',
            name="relu88_1_32_")

        conv89_1_32_ = mx.symbol.Convolution(data=relu88_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv89_1_32_")
        # conv89_1_32_, output shape: {[2048,7,7]}

        batchnorm89_1_32_ = mx.symbol.BatchNorm(data=conv89_1_32_,
            fix_gamma=True,
            name="batchnorm89_1_32_")
        add90_1_ = batchnorm89_1_1_ + batchnorm89_1_2_ + batchnorm89_1_3_ + batchnorm89_1_4_ + batchnorm89_1_5_ + batchnorm89_1_6_ + batchnorm89_1_7_ + batchnorm89_1_8_ + batchnorm89_1_9_ + batchnorm89_1_10_ + batchnorm89_1_11_ + batchnorm89_1_12_ + batchnorm89_1_13_ + batchnorm89_1_14_ + batchnorm89_1_15_ + batchnorm89_1_16_ + batchnorm89_1_17_ + batchnorm89_1_18_ + batchnorm89_1_19_ + batchnorm89_1_20_ + batchnorm89_1_21_ + batchnorm89_1_22_ + batchnorm89_1_23_ + batchnorm89_1_24_ + batchnorm89_1_25_ + batchnorm89_1_26_ + batchnorm89_1_27_ + batchnorm89_1_28_ + batchnorm89_1_29_ + batchnorm89_1_30_ + batchnorm89_1_31_ + batchnorm89_1_32_
        # add90_1_, output shape: {[2048,7,7]}

        add91_ = add90_1_ + relu85_
        # add91_, output shape: {[2048,7,7]}

        relu91_ = mx.symbol.Activation(data=add91_,
            act_type='relu',
            name="relu91_")

        conv93_1_1_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_1_")
        # conv93_1_1_, output shape: {[32,7,7]}

        batchnorm93_1_1_ = mx.symbol.BatchNorm(data=conv93_1_1_,
            fix_gamma=True,
            name="batchnorm93_1_1_")
        relu93_1_1_ = mx.symbol.Activation(data=batchnorm93_1_1_,
            act_type='relu',
            name="relu93_1_1_")

        conv94_1_1_ = mx.symbol.pad(data=relu93_1_1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_1_ = mx.symbol.Convolution(data=conv94_1_1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_1_")
        # conv94_1_1_, output shape: {[32,7,7]}

        batchnorm94_1_1_ = mx.symbol.BatchNorm(data=conv94_1_1_,
            fix_gamma=True,
            name="batchnorm94_1_1_")
        relu94_1_1_ = mx.symbol.Activation(data=batchnorm94_1_1_,
            act_type='relu',
            name="relu94_1_1_")

        conv95_1_1_ = mx.symbol.Convolution(data=relu94_1_1_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_1_")
        # conv95_1_1_, output shape: {[2048,7,7]}

        batchnorm95_1_1_ = mx.symbol.BatchNorm(data=conv95_1_1_,
            fix_gamma=True,
            name="batchnorm95_1_1_")
        conv93_1_2_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_2_")
        # conv93_1_2_, output shape: {[32,7,7]}

        batchnorm93_1_2_ = mx.symbol.BatchNorm(data=conv93_1_2_,
            fix_gamma=True,
            name="batchnorm93_1_2_")
        relu93_1_2_ = mx.symbol.Activation(data=batchnorm93_1_2_,
            act_type='relu',
            name="relu93_1_2_")

        conv94_1_2_ = mx.symbol.pad(data=relu93_1_2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_2_ = mx.symbol.Convolution(data=conv94_1_2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_2_")
        # conv94_1_2_, output shape: {[32,7,7]}

        batchnorm94_1_2_ = mx.symbol.BatchNorm(data=conv94_1_2_,
            fix_gamma=True,
            name="batchnorm94_1_2_")
        relu94_1_2_ = mx.symbol.Activation(data=batchnorm94_1_2_,
            act_type='relu',
            name="relu94_1_2_")

        conv95_1_2_ = mx.symbol.Convolution(data=relu94_1_2_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_2_")
        # conv95_1_2_, output shape: {[2048,7,7]}

        batchnorm95_1_2_ = mx.symbol.BatchNorm(data=conv95_1_2_,
            fix_gamma=True,
            name="batchnorm95_1_2_")
        conv93_1_3_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_3_")
        # conv93_1_3_, output shape: {[32,7,7]}

        batchnorm93_1_3_ = mx.symbol.BatchNorm(data=conv93_1_3_,
            fix_gamma=True,
            name="batchnorm93_1_3_")
        relu93_1_3_ = mx.symbol.Activation(data=batchnorm93_1_3_,
            act_type='relu',
            name="relu93_1_3_")

        conv94_1_3_ = mx.symbol.pad(data=relu93_1_3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_3_ = mx.symbol.Convolution(data=conv94_1_3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_3_")
        # conv94_1_3_, output shape: {[32,7,7]}

        batchnorm94_1_3_ = mx.symbol.BatchNorm(data=conv94_1_3_,
            fix_gamma=True,
            name="batchnorm94_1_3_")
        relu94_1_3_ = mx.symbol.Activation(data=batchnorm94_1_3_,
            act_type='relu',
            name="relu94_1_3_")

        conv95_1_3_ = mx.symbol.Convolution(data=relu94_1_3_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_3_")
        # conv95_1_3_, output shape: {[2048,7,7]}

        batchnorm95_1_3_ = mx.symbol.BatchNorm(data=conv95_1_3_,
            fix_gamma=True,
            name="batchnorm95_1_3_")
        conv93_1_4_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_4_")
        # conv93_1_4_, output shape: {[32,7,7]}

        batchnorm93_1_4_ = mx.symbol.BatchNorm(data=conv93_1_4_,
            fix_gamma=True,
            name="batchnorm93_1_4_")
        relu93_1_4_ = mx.symbol.Activation(data=batchnorm93_1_4_,
            act_type='relu',
            name="relu93_1_4_")

        conv94_1_4_ = mx.symbol.pad(data=relu93_1_4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_4_ = mx.symbol.Convolution(data=conv94_1_4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_4_")
        # conv94_1_4_, output shape: {[32,7,7]}

        batchnorm94_1_4_ = mx.symbol.BatchNorm(data=conv94_1_4_,
            fix_gamma=True,
            name="batchnorm94_1_4_")
        relu94_1_4_ = mx.symbol.Activation(data=batchnorm94_1_4_,
            act_type='relu',
            name="relu94_1_4_")

        conv95_1_4_ = mx.symbol.Convolution(data=relu94_1_4_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_4_")
        # conv95_1_4_, output shape: {[2048,7,7]}

        batchnorm95_1_4_ = mx.symbol.BatchNorm(data=conv95_1_4_,
            fix_gamma=True,
            name="batchnorm95_1_4_")
        conv93_1_5_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_5_")
        # conv93_1_5_, output shape: {[32,7,7]}

        batchnorm93_1_5_ = mx.symbol.BatchNorm(data=conv93_1_5_,
            fix_gamma=True,
            name="batchnorm93_1_5_")
        relu93_1_5_ = mx.symbol.Activation(data=batchnorm93_1_5_,
            act_type='relu',
            name="relu93_1_5_")

        conv94_1_5_ = mx.symbol.pad(data=relu93_1_5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_5_ = mx.symbol.Convolution(data=conv94_1_5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_5_")
        # conv94_1_5_, output shape: {[32,7,7]}

        batchnorm94_1_5_ = mx.symbol.BatchNorm(data=conv94_1_5_,
            fix_gamma=True,
            name="batchnorm94_1_5_")
        relu94_1_5_ = mx.symbol.Activation(data=batchnorm94_1_5_,
            act_type='relu',
            name="relu94_1_5_")

        conv95_1_5_ = mx.symbol.Convolution(data=relu94_1_5_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_5_")
        # conv95_1_5_, output shape: {[2048,7,7]}

        batchnorm95_1_5_ = mx.symbol.BatchNorm(data=conv95_1_5_,
            fix_gamma=True,
            name="batchnorm95_1_5_")
        conv93_1_6_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_6_")
        # conv93_1_6_, output shape: {[32,7,7]}

        batchnorm93_1_6_ = mx.symbol.BatchNorm(data=conv93_1_6_,
            fix_gamma=True,
            name="batchnorm93_1_6_")
        relu93_1_6_ = mx.symbol.Activation(data=batchnorm93_1_6_,
            act_type='relu',
            name="relu93_1_6_")

        conv94_1_6_ = mx.symbol.pad(data=relu93_1_6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_6_ = mx.symbol.Convolution(data=conv94_1_6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_6_")
        # conv94_1_6_, output shape: {[32,7,7]}

        batchnorm94_1_6_ = mx.symbol.BatchNorm(data=conv94_1_6_,
            fix_gamma=True,
            name="batchnorm94_1_6_")
        relu94_1_6_ = mx.symbol.Activation(data=batchnorm94_1_6_,
            act_type='relu',
            name="relu94_1_6_")

        conv95_1_6_ = mx.symbol.Convolution(data=relu94_1_6_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_6_")
        # conv95_1_6_, output shape: {[2048,7,7]}

        batchnorm95_1_6_ = mx.symbol.BatchNorm(data=conv95_1_6_,
            fix_gamma=True,
            name="batchnorm95_1_6_")
        conv93_1_7_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_7_")
        # conv93_1_7_, output shape: {[32,7,7]}

        batchnorm93_1_7_ = mx.symbol.BatchNorm(data=conv93_1_7_,
            fix_gamma=True,
            name="batchnorm93_1_7_")
        relu93_1_7_ = mx.symbol.Activation(data=batchnorm93_1_7_,
            act_type='relu',
            name="relu93_1_7_")

        conv94_1_7_ = mx.symbol.pad(data=relu93_1_7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_7_ = mx.symbol.Convolution(data=conv94_1_7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_7_")
        # conv94_1_7_, output shape: {[32,7,7]}

        batchnorm94_1_7_ = mx.symbol.BatchNorm(data=conv94_1_7_,
            fix_gamma=True,
            name="batchnorm94_1_7_")
        relu94_1_7_ = mx.symbol.Activation(data=batchnorm94_1_7_,
            act_type='relu',
            name="relu94_1_7_")

        conv95_1_7_ = mx.symbol.Convolution(data=relu94_1_7_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_7_")
        # conv95_1_7_, output shape: {[2048,7,7]}

        batchnorm95_1_7_ = mx.symbol.BatchNorm(data=conv95_1_7_,
            fix_gamma=True,
            name="batchnorm95_1_7_")
        conv93_1_8_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_8_")
        # conv93_1_8_, output shape: {[32,7,7]}

        batchnorm93_1_8_ = mx.symbol.BatchNorm(data=conv93_1_8_,
            fix_gamma=True,
            name="batchnorm93_1_8_")
        relu93_1_8_ = mx.symbol.Activation(data=batchnorm93_1_8_,
            act_type='relu',
            name="relu93_1_8_")

        conv94_1_8_ = mx.symbol.pad(data=relu93_1_8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_8_ = mx.symbol.Convolution(data=conv94_1_8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_8_")
        # conv94_1_8_, output shape: {[32,7,7]}

        batchnorm94_1_8_ = mx.symbol.BatchNorm(data=conv94_1_8_,
            fix_gamma=True,
            name="batchnorm94_1_8_")
        relu94_1_8_ = mx.symbol.Activation(data=batchnorm94_1_8_,
            act_type='relu',
            name="relu94_1_8_")

        conv95_1_8_ = mx.symbol.Convolution(data=relu94_1_8_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_8_")
        # conv95_1_8_, output shape: {[2048,7,7]}

        batchnorm95_1_8_ = mx.symbol.BatchNorm(data=conv95_1_8_,
            fix_gamma=True,
            name="batchnorm95_1_8_")
        conv93_1_9_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_9_")
        # conv93_1_9_, output shape: {[32,7,7]}

        batchnorm93_1_9_ = mx.symbol.BatchNorm(data=conv93_1_9_,
            fix_gamma=True,
            name="batchnorm93_1_9_")
        relu93_1_9_ = mx.symbol.Activation(data=batchnorm93_1_9_,
            act_type='relu',
            name="relu93_1_9_")

        conv94_1_9_ = mx.symbol.pad(data=relu93_1_9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_9_ = mx.symbol.Convolution(data=conv94_1_9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_9_")
        # conv94_1_9_, output shape: {[32,7,7]}

        batchnorm94_1_9_ = mx.symbol.BatchNorm(data=conv94_1_9_,
            fix_gamma=True,
            name="batchnorm94_1_9_")
        relu94_1_9_ = mx.symbol.Activation(data=batchnorm94_1_9_,
            act_type='relu',
            name="relu94_1_9_")

        conv95_1_9_ = mx.symbol.Convolution(data=relu94_1_9_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_9_")
        # conv95_1_9_, output shape: {[2048,7,7]}

        batchnorm95_1_9_ = mx.symbol.BatchNorm(data=conv95_1_9_,
            fix_gamma=True,
            name="batchnorm95_1_9_")
        conv93_1_10_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_10_")
        # conv93_1_10_, output shape: {[32,7,7]}

        batchnorm93_1_10_ = mx.symbol.BatchNorm(data=conv93_1_10_,
            fix_gamma=True,
            name="batchnorm93_1_10_")
        relu93_1_10_ = mx.symbol.Activation(data=batchnorm93_1_10_,
            act_type='relu',
            name="relu93_1_10_")

        conv94_1_10_ = mx.symbol.pad(data=relu93_1_10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_10_ = mx.symbol.Convolution(data=conv94_1_10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_10_")
        # conv94_1_10_, output shape: {[32,7,7]}

        batchnorm94_1_10_ = mx.symbol.BatchNorm(data=conv94_1_10_,
            fix_gamma=True,
            name="batchnorm94_1_10_")
        relu94_1_10_ = mx.symbol.Activation(data=batchnorm94_1_10_,
            act_type='relu',
            name="relu94_1_10_")

        conv95_1_10_ = mx.symbol.Convolution(data=relu94_1_10_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_10_")
        # conv95_1_10_, output shape: {[2048,7,7]}

        batchnorm95_1_10_ = mx.symbol.BatchNorm(data=conv95_1_10_,
            fix_gamma=True,
            name="batchnorm95_1_10_")
        conv93_1_11_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_11_")
        # conv93_1_11_, output shape: {[32,7,7]}

        batchnorm93_1_11_ = mx.symbol.BatchNorm(data=conv93_1_11_,
            fix_gamma=True,
            name="batchnorm93_1_11_")
        relu93_1_11_ = mx.symbol.Activation(data=batchnorm93_1_11_,
            act_type='relu',
            name="relu93_1_11_")

        conv94_1_11_ = mx.symbol.pad(data=relu93_1_11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_11_ = mx.symbol.Convolution(data=conv94_1_11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_11_")
        # conv94_1_11_, output shape: {[32,7,7]}

        batchnorm94_1_11_ = mx.symbol.BatchNorm(data=conv94_1_11_,
            fix_gamma=True,
            name="batchnorm94_1_11_")
        relu94_1_11_ = mx.symbol.Activation(data=batchnorm94_1_11_,
            act_type='relu',
            name="relu94_1_11_")

        conv95_1_11_ = mx.symbol.Convolution(data=relu94_1_11_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_11_")
        # conv95_1_11_, output shape: {[2048,7,7]}

        batchnorm95_1_11_ = mx.symbol.BatchNorm(data=conv95_1_11_,
            fix_gamma=True,
            name="batchnorm95_1_11_")
        conv93_1_12_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_12_")
        # conv93_1_12_, output shape: {[32,7,7]}

        batchnorm93_1_12_ = mx.symbol.BatchNorm(data=conv93_1_12_,
            fix_gamma=True,
            name="batchnorm93_1_12_")
        relu93_1_12_ = mx.symbol.Activation(data=batchnorm93_1_12_,
            act_type='relu',
            name="relu93_1_12_")

        conv94_1_12_ = mx.symbol.pad(data=relu93_1_12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_12_ = mx.symbol.Convolution(data=conv94_1_12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_12_")
        # conv94_1_12_, output shape: {[32,7,7]}

        batchnorm94_1_12_ = mx.symbol.BatchNorm(data=conv94_1_12_,
            fix_gamma=True,
            name="batchnorm94_1_12_")
        relu94_1_12_ = mx.symbol.Activation(data=batchnorm94_1_12_,
            act_type='relu',
            name="relu94_1_12_")

        conv95_1_12_ = mx.symbol.Convolution(data=relu94_1_12_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_12_")
        # conv95_1_12_, output shape: {[2048,7,7]}

        batchnorm95_1_12_ = mx.symbol.BatchNorm(data=conv95_1_12_,
            fix_gamma=True,
            name="batchnorm95_1_12_")
        conv93_1_13_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_13_")
        # conv93_1_13_, output shape: {[32,7,7]}

        batchnorm93_1_13_ = mx.symbol.BatchNorm(data=conv93_1_13_,
            fix_gamma=True,
            name="batchnorm93_1_13_")
        relu93_1_13_ = mx.symbol.Activation(data=batchnorm93_1_13_,
            act_type='relu',
            name="relu93_1_13_")

        conv94_1_13_ = mx.symbol.pad(data=relu93_1_13_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_13_ = mx.symbol.Convolution(data=conv94_1_13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_13_")
        # conv94_1_13_, output shape: {[32,7,7]}

        batchnorm94_1_13_ = mx.symbol.BatchNorm(data=conv94_1_13_,
            fix_gamma=True,
            name="batchnorm94_1_13_")
        relu94_1_13_ = mx.symbol.Activation(data=batchnorm94_1_13_,
            act_type='relu',
            name="relu94_1_13_")

        conv95_1_13_ = mx.symbol.Convolution(data=relu94_1_13_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_13_")
        # conv95_1_13_, output shape: {[2048,7,7]}

        batchnorm95_1_13_ = mx.symbol.BatchNorm(data=conv95_1_13_,
            fix_gamma=True,
            name="batchnorm95_1_13_")
        conv93_1_14_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_14_")
        # conv93_1_14_, output shape: {[32,7,7]}

        batchnorm93_1_14_ = mx.symbol.BatchNorm(data=conv93_1_14_,
            fix_gamma=True,
            name="batchnorm93_1_14_")
        relu93_1_14_ = mx.symbol.Activation(data=batchnorm93_1_14_,
            act_type='relu',
            name="relu93_1_14_")

        conv94_1_14_ = mx.symbol.pad(data=relu93_1_14_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_14_ = mx.symbol.Convolution(data=conv94_1_14_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_14_")
        # conv94_1_14_, output shape: {[32,7,7]}

        batchnorm94_1_14_ = mx.symbol.BatchNorm(data=conv94_1_14_,
            fix_gamma=True,
            name="batchnorm94_1_14_")
        relu94_1_14_ = mx.symbol.Activation(data=batchnorm94_1_14_,
            act_type='relu',
            name="relu94_1_14_")

        conv95_1_14_ = mx.symbol.Convolution(data=relu94_1_14_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_14_")
        # conv95_1_14_, output shape: {[2048,7,7]}

        batchnorm95_1_14_ = mx.symbol.BatchNorm(data=conv95_1_14_,
            fix_gamma=True,
            name="batchnorm95_1_14_")
        conv93_1_15_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_15_")
        # conv93_1_15_, output shape: {[32,7,7]}

        batchnorm93_1_15_ = mx.symbol.BatchNorm(data=conv93_1_15_,
            fix_gamma=True,
            name="batchnorm93_1_15_")
        relu93_1_15_ = mx.symbol.Activation(data=batchnorm93_1_15_,
            act_type='relu',
            name="relu93_1_15_")

        conv94_1_15_ = mx.symbol.pad(data=relu93_1_15_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_15_ = mx.symbol.Convolution(data=conv94_1_15_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_15_")
        # conv94_1_15_, output shape: {[32,7,7]}

        batchnorm94_1_15_ = mx.symbol.BatchNorm(data=conv94_1_15_,
            fix_gamma=True,
            name="batchnorm94_1_15_")
        relu94_1_15_ = mx.symbol.Activation(data=batchnorm94_1_15_,
            act_type='relu',
            name="relu94_1_15_")

        conv95_1_15_ = mx.symbol.Convolution(data=relu94_1_15_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_15_")
        # conv95_1_15_, output shape: {[2048,7,7]}

        batchnorm95_1_15_ = mx.symbol.BatchNorm(data=conv95_1_15_,
            fix_gamma=True,
            name="batchnorm95_1_15_")
        conv93_1_16_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_16_")
        # conv93_1_16_, output shape: {[32,7,7]}

        batchnorm93_1_16_ = mx.symbol.BatchNorm(data=conv93_1_16_,
            fix_gamma=True,
            name="batchnorm93_1_16_")
        relu93_1_16_ = mx.symbol.Activation(data=batchnorm93_1_16_,
            act_type='relu',
            name="relu93_1_16_")

        conv94_1_16_ = mx.symbol.pad(data=relu93_1_16_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_16_ = mx.symbol.Convolution(data=conv94_1_16_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_16_")
        # conv94_1_16_, output shape: {[32,7,7]}

        batchnorm94_1_16_ = mx.symbol.BatchNorm(data=conv94_1_16_,
            fix_gamma=True,
            name="batchnorm94_1_16_")
        relu94_1_16_ = mx.symbol.Activation(data=batchnorm94_1_16_,
            act_type='relu',
            name="relu94_1_16_")

        conv95_1_16_ = mx.symbol.Convolution(data=relu94_1_16_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_16_")
        # conv95_1_16_, output shape: {[2048,7,7]}

        batchnorm95_1_16_ = mx.symbol.BatchNorm(data=conv95_1_16_,
            fix_gamma=True,
            name="batchnorm95_1_16_")
        conv93_1_17_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_17_")
        # conv93_1_17_, output shape: {[32,7,7]}

        batchnorm93_1_17_ = mx.symbol.BatchNorm(data=conv93_1_17_,
            fix_gamma=True,
            name="batchnorm93_1_17_")
        relu93_1_17_ = mx.symbol.Activation(data=batchnorm93_1_17_,
            act_type='relu',
            name="relu93_1_17_")

        conv94_1_17_ = mx.symbol.pad(data=relu93_1_17_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_17_ = mx.symbol.Convolution(data=conv94_1_17_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_17_")
        # conv94_1_17_, output shape: {[32,7,7]}

        batchnorm94_1_17_ = mx.symbol.BatchNorm(data=conv94_1_17_,
            fix_gamma=True,
            name="batchnorm94_1_17_")
        relu94_1_17_ = mx.symbol.Activation(data=batchnorm94_1_17_,
            act_type='relu',
            name="relu94_1_17_")

        conv95_1_17_ = mx.symbol.Convolution(data=relu94_1_17_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_17_")
        # conv95_1_17_, output shape: {[2048,7,7]}

        batchnorm95_1_17_ = mx.symbol.BatchNorm(data=conv95_1_17_,
            fix_gamma=True,
            name="batchnorm95_1_17_")
        conv93_1_18_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_18_")
        # conv93_1_18_, output shape: {[32,7,7]}

        batchnorm93_1_18_ = mx.symbol.BatchNorm(data=conv93_1_18_,
            fix_gamma=True,
            name="batchnorm93_1_18_")
        relu93_1_18_ = mx.symbol.Activation(data=batchnorm93_1_18_,
            act_type='relu',
            name="relu93_1_18_")

        conv94_1_18_ = mx.symbol.pad(data=relu93_1_18_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_18_ = mx.symbol.Convolution(data=conv94_1_18_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_18_")
        # conv94_1_18_, output shape: {[32,7,7]}

        batchnorm94_1_18_ = mx.symbol.BatchNorm(data=conv94_1_18_,
            fix_gamma=True,
            name="batchnorm94_1_18_")
        relu94_1_18_ = mx.symbol.Activation(data=batchnorm94_1_18_,
            act_type='relu',
            name="relu94_1_18_")

        conv95_1_18_ = mx.symbol.Convolution(data=relu94_1_18_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_18_")
        # conv95_1_18_, output shape: {[2048,7,7]}

        batchnorm95_1_18_ = mx.symbol.BatchNorm(data=conv95_1_18_,
            fix_gamma=True,
            name="batchnorm95_1_18_")
        conv93_1_19_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_19_")
        # conv93_1_19_, output shape: {[32,7,7]}

        batchnorm93_1_19_ = mx.symbol.BatchNorm(data=conv93_1_19_,
            fix_gamma=True,
            name="batchnorm93_1_19_")
        relu93_1_19_ = mx.symbol.Activation(data=batchnorm93_1_19_,
            act_type='relu',
            name="relu93_1_19_")

        conv94_1_19_ = mx.symbol.pad(data=relu93_1_19_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_19_ = mx.symbol.Convolution(data=conv94_1_19_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_19_")
        # conv94_1_19_, output shape: {[32,7,7]}

        batchnorm94_1_19_ = mx.symbol.BatchNorm(data=conv94_1_19_,
            fix_gamma=True,
            name="batchnorm94_1_19_")
        relu94_1_19_ = mx.symbol.Activation(data=batchnorm94_1_19_,
            act_type='relu',
            name="relu94_1_19_")

        conv95_1_19_ = mx.symbol.Convolution(data=relu94_1_19_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_19_")
        # conv95_1_19_, output shape: {[2048,7,7]}

        batchnorm95_1_19_ = mx.symbol.BatchNorm(data=conv95_1_19_,
            fix_gamma=True,
            name="batchnorm95_1_19_")
        conv93_1_20_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_20_")
        # conv93_1_20_, output shape: {[32,7,7]}

        batchnorm93_1_20_ = mx.symbol.BatchNorm(data=conv93_1_20_,
            fix_gamma=True,
            name="batchnorm93_1_20_")
        relu93_1_20_ = mx.symbol.Activation(data=batchnorm93_1_20_,
            act_type='relu',
            name="relu93_1_20_")

        conv94_1_20_ = mx.symbol.pad(data=relu93_1_20_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_20_ = mx.symbol.Convolution(data=conv94_1_20_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_20_")
        # conv94_1_20_, output shape: {[32,7,7]}

        batchnorm94_1_20_ = mx.symbol.BatchNorm(data=conv94_1_20_,
            fix_gamma=True,
            name="batchnorm94_1_20_")
        relu94_1_20_ = mx.symbol.Activation(data=batchnorm94_1_20_,
            act_type='relu',
            name="relu94_1_20_")

        conv95_1_20_ = mx.symbol.Convolution(data=relu94_1_20_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_20_")
        # conv95_1_20_, output shape: {[2048,7,7]}

        batchnorm95_1_20_ = mx.symbol.BatchNorm(data=conv95_1_20_,
            fix_gamma=True,
            name="batchnorm95_1_20_")
        conv93_1_21_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_21_")
        # conv93_1_21_, output shape: {[32,7,7]}

        batchnorm93_1_21_ = mx.symbol.BatchNorm(data=conv93_1_21_,
            fix_gamma=True,
            name="batchnorm93_1_21_")
        relu93_1_21_ = mx.symbol.Activation(data=batchnorm93_1_21_,
            act_type='relu',
            name="relu93_1_21_")

        conv94_1_21_ = mx.symbol.pad(data=relu93_1_21_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_21_ = mx.symbol.Convolution(data=conv94_1_21_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_21_")
        # conv94_1_21_, output shape: {[32,7,7]}

        batchnorm94_1_21_ = mx.symbol.BatchNorm(data=conv94_1_21_,
            fix_gamma=True,
            name="batchnorm94_1_21_")
        relu94_1_21_ = mx.symbol.Activation(data=batchnorm94_1_21_,
            act_type='relu',
            name="relu94_1_21_")

        conv95_1_21_ = mx.symbol.Convolution(data=relu94_1_21_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_21_")
        # conv95_1_21_, output shape: {[2048,7,7]}

        batchnorm95_1_21_ = mx.symbol.BatchNorm(data=conv95_1_21_,
            fix_gamma=True,
            name="batchnorm95_1_21_")
        conv93_1_22_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_22_")
        # conv93_1_22_, output shape: {[32,7,7]}

        batchnorm93_1_22_ = mx.symbol.BatchNorm(data=conv93_1_22_,
            fix_gamma=True,
            name="batchnorm93_1_22_")
        relu93_1_22_ = mx.symbol.Activation(data=batchnorm93_1_22_,
            act_type='relu',
            name="relu93_1_22_")

        conv94_1_22_ = mx.symbol.pad(data=relu93_1_22_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_22_ = mx.symbol.Convolution(data=conv94_1_22_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_22_")
        # conv94_1_22_, output shape: {[32,7,7]}

        batchnorm94_1_22_ = mx.symbol.BatchNorm(data=conv94_1_22_,
            fix_gamma=True,
            name="batchnorm94_1_22_")
        relu94_1_22_ = mx.symbol.Activation(data=batchnorm94_1_22_,
            act_type='relu',
            name="relu94_1_22_")

        conv95_1_22_ = mx.symbol.Convolution(data=relu94_1_22_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_22_")
        # conv95_1_22_, output shape: {[2048,7,7]}

        batchnorm95_1_22_ = mx.symbol.BatchNorm(data=conv95_1_22_,
            fix_gamma=True,
            name="batchnorm95_1_22_")
        conv93_1_23_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_23_")
        # conv93_1_23_, output shape: {[32,7,7]}

        batchnorm93_1_23_ = mx.symbol.BatchNorm(data=conv93_1_23_,
            fix_gamma=True,
            name="batchnorm93_1_23_")
        relu93_1_23_ = mx.symbol.Activation(data=batchnorm93_1_23_,
            act_type='relu',
            name="relu93_1_23_")

        conv94_1_23_ = mx.symbol.pad(data=relu93_1_23_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_23_ = mx.symbol.Convolution(data=conv94_1_23_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_23_")
        # conv94_1_23_, output shape: {[32,7,7]}

        batchnorm94_1_23_ = mx.symbol.BatchNorm(data=conv94_1_23_,
            fix_gamma=True,
            name="batchnorm94_1_23_")
        relu94_1_23_ = mx.symbol.Activation(data=batchnorm94_1_23_,
            act_type='relu',
            name="relu94_1_23_")

        conv95_1_23_ = mx.symbol.Convolution(data=relu94_1_23_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_23_")
        # conv95_1_23_, output shape: {[2048,7,7]}

        batchnorm95_1_23_ = mx.symbol.BatchNorm(data=conv95_1_23_,
            fix_gamma=True,
            name="batchnorm95_1_23_")
        conv93_1_24_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_24_")
        # conv93_1_24_, output shape: {[32,7,7]}

        batchnorm93_1_24_ = mx.symbol.BatchNorm(data=conv93_1_24_,
            fix_gamma=True,
            name="batchnorm93_1_24_")
        relu93_1_24_ = mx.symbol.Activation(data=batchnorm93_1_24_,
            act_type='relu',
            name="relu93_1_24_")

        conv94_1_24_ = mx.symbol.pad(data=relu93_1_24_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_24_ = mx.symbol.Convolution(data=conv94_1_24_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_24_")
        # conv94_1_24_, output shape: {[32,7,7]}

        batchnorm94_1_24_ = mx.symbol.BatchNorm(data=conv94_1_24_,
            fix_gamma=True,
            name="batchnorm94_1_24_")
        relu94_1_24_ = mx.symbol.Activation(data=batchnorm94_1_24_,
            act_type='relu',
            name="relu94_1_24_")

        conv95_1_24_ = mx.symbol.Convolution(data=relu94_1_24_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_24_")
        # conv95_1_24_, output shape: {[2048,7,7]}

        batchnorm95_1_24_ = mx.symbol.BatchNorm(data=conv95_1_24_,
            fix_gamma=True,
            name="batchnorm95_1_24_")
        conv93_1_25_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_25_")
        # conv93_1_25_, output shape: {[32,7,7]}

        batchnorm93_1_25_ = mx.symbol.BatchNorm(data=conv93_1_25_,
            fix_gamma=True,
            name="batchnorm93_1_25_")
        relu93_1_25_ = mx.symbol.Activation(data=batchnorm93_1_25_,
            act_type='relu',
            name="relu93_1_25_")

        conv94_1_25_ = mx.symbol.pad(data=relu93_1_25_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_25_ = mx.symbol.Convolution(data=conv94_1_25_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_25_")
        # conv94_1_25_, output shape: {[32,7,7]}

        batchnorm94_1_25_ = mx.symbol.BatchNorm(data=conv94_1_25_,
            fix_gamma=True,
            name="batchnorm94_1_25_")
        relu94_1_25_ = mx.symbol.Activation(data=batchnorm94_1_25_,
            act_type='relu',
            name="relu94_1_25_")

        conv95_1_25_ = mx.symbol.Convolution(data=relu94_1_25_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_25_")
        # conv95_1_25_, output shape: {[2048,7,7]}

        batchnorm95_1_25_ = mx.symbol.BatchNorm(data=conv95_1_25_,
            fix_gamma=True,
            name="batchnorm95_1_25_")
        conv93_1_26_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_26_")
        # conv93_1_26_, output shape: {[32,7,7]}

        batchnorm93_1_26_ = mx.symbol.BatchNorm(data=conv93_1_26_,
            fix_gamma=True,
            name="batchnorm93_1_26_")
        relu93_1_26_ = mx.symbol.Activation(data=batchnorm93_1_26_,
            act_type='relu',
            name="relu93_1_26_")

        conv94_1_26_ = mx.symbol.pad(data=relu93_1_26_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_26_ = mx.symbol.Convolution(data=conv94_1_26_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_26_")
        # conv94_1_26_, output shape: {[32,7,7]}

        batchnorm94_1_26_ = mx.symbol.BatchNorm(data=conv94_1_26_,
            fix_gamma=True,
            name="batchnorm94_1_26_")
        relu94_1_26_ = mx.symbol.Activation(data=batchnorm94_1_26_,
            act_type='relu',
            name="relu94_1_26_")

        conv95_1_26_ = mx.symbol.Convolution(data=relu94_1_26_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_26_")
        # conv95_1_26_, output shape: {[2048,7,7]}

        batchnorm95_1_26_ = mx.symbol.BatchNorm(data=conv95_1_26_,
            fix_gamma=True,
            name="batchnorm95_1_26_")
        conv93_1_27_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_27_")
        # conv93_1_27_, output shape: {[32,7,7]}

        batchnorm93_1_27_ = mx.symbol.BatchNorm(data=conv93_1_27_,
            fix_gamma=True,
            name="batchnorm93_1_27_")
        relu93_1_27_ = mx.symbol.Activation(data=batchnorm93_1_27_,
            act_type='relu',
            name="relu93_1_27_")

        conv94_1_27_ = mx.symbol.pad(data=relu93_1_27_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_27_ = mx.symbol.Convolution(data=conv94_1_27_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_27_")
        # conv94_1_27_, output shape: {[32,7,7]}

        batchnorm94_1_27_ = mx.symbol.BatchNorm(data=conv94_1_27_,
            fix_gamma=True,
            name="batchnorm94_1_27_")
        relu94_1_27_ = mx.symbol.Activation(data=batchnorm94_1_27_,
            act_type='relu',
            name="relu94_1_27_")

        conv95_1_27_ = mx.symbol.Convolution(data=relu94_1_27_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_27_")
        # conv95_1_27_, output shape: {[2048,7,7]}

        batchnorm95_1_27_ = mx.symbol.BatchNorm(data=conv95_1_27_,
            fix_gamma=True,
            name="batchnorm95_1_27_")
        conv93_1_28_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_28_")
        # conv93_1_28_, output shape: {[32,7,7]}

        batchnorm93_1_28_ = mx.symbol.BatchNorm(data=conv93_1_28_,
            fix_gamma=True,
            name="batchnorm93_1_28_")
        relu93_1_28_ = mx.symbol.Activation(data=batchnorm93_1_28_,
            act_type='relu',
            name="relu93_1_28_")

        conv94_1_28_ = mx.symbol.pad(data=relu93_1_28_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_28_ = mx.symbol.Convolution(data=conv94_1_28_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_28_")
        # conv94_1_28_, output shape: {[32,7,7]}

        batchnorm94_1_28_ = mx.symbol.BatchNorm(data=conv94_1_28_,
            fix_gamma=True,
            name="batchnorm94_1_28_")
        relu94_1_28_ = mx.symbol.Activation(data=batchnorm94_1_28_,
            act_type='relu',
            name="relu94_1_28_")

        conv95_1_28_ = mx.symbol.Convolution(data=relu94_1_28_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_28_")
        # conv95_1_28_, output shape: {[2048,7,7]}

        batchnorm95_1_28_ = mx.symbol.BatchNorm(data=conv95_1_28_,
            fix_gamma=True,
            name="batchnorm95_1_28_")
        conv93_1_29_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_29_")
        # conv93_1_29_, output shape: {[32,7,7]}

        batchnorm93_1_29_ = mx.symbol.BatchNorm(data=conv93_1_29_,
            fix_gamma=True,
            name="batchnorm93_1_29_")
        relu93_1_29_ = mx.symbol.Activation(data=batchnorm93_1_29_,
            act_type='relu',
            name="relu93_1_29_")

        conv94_1_29_ = mx.symbol.pad(data=relu93_1_29_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_29_ = mx.symbol.Convolution(data=conv94_1_29_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_29_")
        # conv94_1_29_, output shape: {[32,7,7]}

        batchnorm94_1_29_ = mx.symbol.BatchNorm(data=conv94_1_29_,
            fix_gamma=True,
            name="batchnorm94_1_29_")
        relu94_1_29_ = mx.symbol.Activation(data=batchnorm94_1_29_,
            act_type='relu',
            name="relu94_1_29_")

        conv95_1_29_ = mx.symbol.Convolution(data=relu94_1_29_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_29_")
        # conv95_1_29_, output shape: {[2048,7,7]}

        batchnorm95_1_29_ = mx.symbol.BatchNorm(data=conv95_1_29_,
            fix_gamma=True,
            name="batchnorm95_1_29_")
        conv93_1_30_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_30_")
        # conv93_1_30_, output shape: {[32,7,7]}

        batchnorm93_1_30_ = mx.symbol.BatchNorm(data=conv93_1_30_,
            fix_gamma=True,
            name="batchnorm93_1_30_")
        relu93_1_30_ = mx.symbol.Activation(data=batchnorm93_1_30_,
            act_type='relu',
            name="relu93_1_30_")

        conv94_1_30_ = mx.symbol.pad(data=relu93_1_30_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_30_ = mx.symbol.Convolution(data=conv94_1_30_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_30_")
        # conv94_1_30_, output shape: {[32,7,7]}

        batchnorm94_1_30_ = mx.symbol.BatchNorm(data=conv94_1_30_,
            fix_gamma=True,
            name="batchnorm94_1_30_")
        relu94_1_30_ = mx.symbol.Activation(data=batchnorm94_1_30_,
            act_type='relu',
            name="relu94_1_30_")

        conv95_1_30_ = mx.symbol.Convolution(data=relu94_1_30_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_30_")
        # conv95_1_30_, output shape: {[2048,7,7]}

        batchnorm95_1_30_ = mx.symbol.BatchNorm(data=conv95_1_30_,
            fix_gamma=True,
            name="batchnorm95_1_30_")
        conv93_1_31_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_31_")
        # conv93_1_31_, output shape: {[32,7,7]}

        batchnorm93_1_31_ = mx.symbol.BatchNorm(data=conv93_1_31_,
            fix_gamma=True,
            name="batchnorm93_1_31_")
        relu93_1_31_ = mx.symbol.Activation(data=batchnorm93_1_31_,
            act_type='relu',
            name="relu93_1_31_")

        conv94_1_31_ = mx.symbol.pad(data=relu93_1_31_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_31_ = mx.symbol.Convolution(data=conv94_1_31_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_31_")
        # conv94_1_31_, output shape: {[32,7,7]}

        batchnorm94_1_31_ = mx.symbol.BatchNorm(data=conv94_1_31_,
            fix_gamma=True,
            name="batchnorm94_1_31_")
        relu94_1_31_ = mx.symbol.Activation(data=batchnorm94_1_31_,
            act_type='relu',
            name="relu94_1_31_")

        conv95_1_31_ = mx.symbol.Convolution(data=relu94_1_31_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_31_")
        # conv95_1_31_, output shape: {[2048,7,7]}

        batchnorm95_1_31_ = mx.symbol.BatchNorm(data=conv95_1_31_,
            fix_gamma=True,
            name="batchnorm95_1_31_")
        conv93_1_32_ = mx.symbol.Convolution(data=relu91_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv93_1_32_")
        # conv93_1_32_, output shape: {[32,7,7]}

        batchnorm93_1_32_ = mx.symbol.BatchNorm(data=conv93_1_32_,
            fix_gamma=True,
            name="batchnorm93_1_32_")
        relu93_1_32_ = mx.symbol.Activation(data=batchnorm93_1_32_,
            act_type='relu',
            name="relu93_1_32_")

        conv94_1_32_ = mx.symbol.pad(data=relu93_1_32_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv94_1_32_ = mx.symbol.Convolution(data=conv94_1_32_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=32,
            no_bias=False,
            name="conv94_1_32_")
        # conv94_1_32_, output shape: {[32,7,7]}

        batchnorm94_1_32_ = mx.symbol.BatchNorm(data=conv94_1_32_,
            fix_gamma=True,
            name="batchnorm94_1_32_")
        relu94_1_32_ = mx.symbol.Activation(data=batchnorm94_1_32_,
            act_type='relu',
            name="relu94_1_32_")

        conv95_1_32_ = mx.symbol.Convolution(data=relu94_1_32_,
            kernel=(1,1),
            stride=(1,1),
            num_filter=2048,
            no_bias=False,
            name="conv95_1_32_")
        # conv95_1_32_, output shape: {[2048,7,7]}

        batchnorm95_1_32_ = mx.symbol.BatchNorm(data=conv95_1_32_,
            fix_gamma=True,
            name="batchnorm95_1_32_")
        add96_1_ = batchnorm95_1_1_ + batchnorm95_1_2_ + batchnorm95_1_3_ + batchnorm95_1_4_ + batchnorm95_1_5_ + batchnorm95_1_6_ + batchnorm95_1_7_ + batchnorm95_1_8_ + batchnorm95_1_9_ + batchnorm95_1_10_ + batchnorm95_1_11_ + batchnorm95_1_12_ + batchnorm95_1_13_ + batchnorm95_1_14_ + batchnorm95_1_15_ + batchnorm95_1_16_ + batchnorm95_1_17_ + batchnorm95_1_18_ + batchnorm95_1_19_ + batchnorm95_1_20_ + batchnorm95_1_21_ + batchnorm95_1_22_ + batchnorm95_1_23_ + batchnorm95_1_24_ + batchnorm95_1_25_ + batchnorm95_1_26_ + batchnorm95_1_27_ + batchnorm95_1_28_ + batchnorm95_1_29_ + batchnorm95_1_30_ + batchnorm95_1_31_ + batchnorm95_1_32_
        # add96_1_, output shape: {[2048,7,7]}

        add97_ = add96_1_ + relu91_
        # add97_, output shape: {[2048,7,7]}

        relu97_ = mx.symbol.Activation(data=add97_,
            act_type='relu',
            name="relu97_")

        globalpooling97_ = mx.symbol.Pooling(data=relu97_,
            global_pool=True,
            kernel=(1,1),
            pool_type="avg",
            name="globalpooling97_")
        # globalpooling97_, output shape: {[2048,1,1]}

        fc97_ = mx.symbol.FullyConnected(data=globalpooling97_,
            num_hidden=1000,
            no_bias=False,
            name="fc97_")

        softmax97_ = mx.symbol.softmax(data=fc97_,
            axis=-1,
            name="softmax97_"
        )
        predictions_ = mx.symbol.SoftmaxOutput(data=softmax97_,
            name="predictions_")
        

        self.module = mx.mod.Module(symbol=mx.symbol.Group([predictions_]),
                                         data_names=self._input_names_,
                                         label_names=self._output_names_,
                                         context=context)
