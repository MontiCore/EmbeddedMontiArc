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

class CNNCreator_VGG16:

    def __init__(self, data_cleaner):
        self.module = None
        self._data_dir_ = "data/VGG16/"
        self._model_dir_ = "model/VGG16/"
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
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv1_ = mx.symbol.Convolution(data=conv1_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=64,
            no_bias=False,
            name="conv1_")
        # conv1_, output shape: {[64,224,224]}

        relu1_ = mx.symbol.Activation(data=conv1_,
            act_type='relu',
            name="relu1_")

        conv2_ = mx.symbol.pad(data=relu1_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv2_ = mx.symbol.Convolution(data=conv2_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=64,
            no_bias=False,
            name="conv2_")
        # conv2_, output shape: {[64,224,224]}

        relu2_ = mx.symbol.Activation(data=conv2_,
            act_type='relu',
            name="relu2_")

        pool2_ = mx.symbol.Pooling(data=relu2_,
            kernel=(2,2),
            pool_type="max",
            stride=(2,2),
            name="pool2_")
        # pool2_, output shape: {[64,112,112]}

        conv3_ = mx.symbol.pad(data=pool2_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv3_ = mx.symbol.Convolution(data=conv3_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=128,
            no_bias=False,
            name="conv3_")
        # conv3_, output shape: {[128,112,112]}

        relu3_ = mx.symbol.Activation(data=conv3_,
            act_type='relu',
            name="relu3_")

        conv4_ = mx.symbol.pad(data=relu3_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv4_ = mx.symbol.Convolution(data=conv4_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=128,
            no_bias=False,
            name="conv4_")
        # conv4_, output shape: {[128,112,112]}

        relu4_ = mx.symbol.Activation(data=conv4_,
            act_type='relu',
            name="relu4_")

        pool4_ = mx.symbol.Pooling(data=relu4_,
            kernel=(2,2),
            pool_type="max",
            stride=(2,2),
            name="pool4_")
        # pool4_, output shape: {[128,56,56]}

        conv5_ = mx.symbol.pad(data=pool4_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv5_ = mx.symbol.Convolution(data=conv5_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv5_")
        # conv5_, output shape: {[256,56,56]}

        relu5_ = mx.symbol.Activation(data=conv5_,
            act_type='relu',
            name="relu5_")

        conv6_ = mx.symbol.pad(data=relu5_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv6_ = mx.symbol.Convolution(data=conv6_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv6_")
        # conv6_, output shape: {[256,56,56]}

        relu6_ = mx.symbol.Activation(data=conv6_,
            act_type='relu',
            name="relu6_")

        conv7_ = mx.symbol.pad(data=relu6_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv7_ = mx.symbol.Convolution(data=conv7_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=256,
            no_bias=False,
            name="conv7_")
        # conv7_, output shape: {[256,56,56]}

        relu7_ = mx.symbol.Activation(data=conv7_,
            act_type='relu',
            name="relu7_")

        pool7_ = mx.symbol.Pooling(data=relu7_,
            kernel=(2,2),
            pool_type="max",
            stride=(2,2),
            name="pool7_")
        # pool7_, output shape: {[256,28,28]}

        conv8_ = mx.symbol.pad(data=pool7_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv8_ = mx.symbol.Convolution(data=conv8_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv8_")
        # conv8_, output shape: {[512,28,28]}

        relu8_ = mx.symbol.Activation(data=conv8_,
            act_type='relu',
            name="relu8_")

        conv9_ = mx.symbol.pad(data=relu8_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv9_ = mx.symbol.Convolution(data=conv9_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv9_")
        # conv9_, output shape: {[512,28,28]}

        relu9_ = mx.symbol.Activation(data=conv9_,
            act_type='relu',
            name="relu9_")

        conv10_ = mx.symbol.pad(data=relu9_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv10_ = mx.symbol.Convolution(data=conv10_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv10_")
        # conv10_, output shape: {[512,28,28]}

        relu10_ = mx.symbol.Activation(data=conv10_,
            act_type='relu',
            name="relu10_")

        pool10_ = mx.symbol.Pooling(data=relu10_,
            kernel=(2,2),
            pool_type="max",
            stride=(2,2),
            name="pool10_")
        # pool10_, output shape: {[512,14,14]}

        conv11_ = mx.symbol.pad(data=pool10_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv11_ = mx.symbol.Convolution(data=conv11_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv11_")
        # conv11_, output shape: {[512,14,14]}

        relu11_ = mx.symbol.Activation(data=conv11_,
            act_type='relu',
            name="relu11_")

        conv12_ = mx.symbol.pad(data=relu11_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv12_ = mx.symbol.Convolution(data=conv12_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv12_")
        # conv12_, output shape: {[512,14,14]}

        relu12_ = mx.symbol.Activation(data=conv12_,
            act_type='relu',
            name="relu12_")

        conv13_ = mx.symbol.pad(data=relu12_,
            mode='constant',
            pad_width=(0,0,0,0,1,1,1,1),
            constant_value=0)
        conv13_ = mx.symbol.Convolution(data=conv13_,
            kernel=(3,3),
            stride=(1,1),
            num_filter=512,
            no_bias=False,
            name="conv13_")
        # conv13_, output shape: {[512,14,14]}

        relu13_ = mx.symbol.Activation(data=conv13_,
            act_type='relu',
            name="relu13_")

        pool13_ = mx.symbol.Pooling(data=relu13_,
            kernel=(2,2),
            pool_type="max",
            stride=(2,2),
            name="pool13_")
        # pool13_, output shape: {[512,7,7]}

        fc13_ = mx.symbol.flatten(data=pool13_)
        fc13_ = mx.symbol.FullyConnected(data=fc13_,
            num_hidden=4096,
            no_bias=False,
            name="fc13_")
        relu14_ = mx.symbol.Activation(data=fc13_,
            act_type='relu',
            name="relu14_")

        dropout14_ = mx.symbol.Dropout(data=relu14_,
            name="dropout14_")
        fc14_ = mx.symbol.FullyConnected(data=dropout14_,
            num_hidden=4096,
            no_bias=False,
            name="fc14_")
        relu15_ = mx.symbol.Activation(data=fc14_,
            act_type='relu',
            name="relu15_")

        dropout15_ = mx.symbol.Dropout(data=relu15_,
            name="dropout15_")
        fc15_ = mx.symbol.FullyConnected(data=dropout15_,
            num_hidden=1000,
            no_bias=False,
            name="fc15_")

        softmax15_ = mx.symbol.softmax(data=fc15_,
            axis=-1,
            name="softmax15_"
        )
        predictions_ = mx.symbol.SoftmaxOutput(data=softmax15_,
            name="predictions_")
        

        self.module = mx.mod.Module(symbol=mx.symbol.Group([predictions_]),
                                         data_names=self._input_names_,
                                         label_names=self._output_names_,
                                         context=context)
