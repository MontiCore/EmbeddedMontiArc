import mxnet as mx
import numpy as np
from mxnet import gluon


class ZScoreNormalization(gluon.HybridBlock):
    def __init__(self, data_mean, data_std, **kwargs):
        super(ZScoreNormalization, self).__init__(**kwargs)
        with self.name_scope():
            self.data_mean = self.params.get('data_mean', shape=data_mean.shape,
                init=mx.init.Constant(data_mean.asnumpy().tolist()), differentiable=False)
            self.data_std = self.params.get('data_std', shape=data_mean.shape,
                init=mx.init.Constant(data_std.asnumpy().tolist()), differentiable=False)

    def hybrid_forward(self, F, x, data_mean, data_std):
        x = F.broadcast_sub(x, data_mean)
        x = F.broadcast_div(x, data_std)
        return x


class Padding(gluon.HybridBlock):
    def __init__(self, padding, **kwargs):
        super(Padding, self).__init__(**kwargs)
        with self.name_scope():
            self.pad_width = padding

    def hybrid_forward(self, F, x):
        x = F.pad(data=x,
            mode='constant',
            pad_width=self.pad_width,
            constant_value=0)
        return x


class NoNormalization(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(NoNormalization, self).__init__(**kwargs)

    def hybrid_forward(self, F, x):
        return x


class Reshape(gluon.HybridBlock):
    def __init__(self, shape, **kwargs):
        super(Reshape, self).__init__(**kwargs)
        with self.name_scope():
            self.shape = shape

    def hybrid_forward(self, F, x):
        return F.reshape(data=x, shape=self.shape)


class CustomRNN(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, bidirectional, **kwargs):
        super(CustomRNN, self).__init__(**kwargs)
        with self.name_scope():
            self.rnn = gluon.rnn.RNN(hidden_size=hidden_size, num_layers=num_layers,
                                     bidirectional=bidirectional, activation='tanh', layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.rnn(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)


class CustomLSTM(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, bidirectional, **kwargs):
        super(CustomLSTM, self).__init__(**kwargs)
        with self.name_scope():
            self.lstm = gluon.rnn.LSTM(hidden_size=hidden_size, num_layers=num_layers,
                                       bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0, state1):
        output, [state0, state1] = self.lstm(data, [F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)])
        return output, F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)


class CustomGRU(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, bidirectional, **kwargs):
        super(CustomGRU, self).__init__(**kwargs)
        with self.name_scope():
            self.gru = gluon.rnn.GRU(hidden_size=hidden_size, num_layers=num_layers,
                                     bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.gru(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)


class Net_0(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_0, self).__init__(**kwargs)
        with self.name_scope():
            if data_mean:
                assert(data_std)
                self.input_normalization_data_ = ZScoreNormalization(data_mean=data_mean['data_'],
                                                                               data_std=data_std['data_'])
            else:
                self.input_normalization_data_ = NoNormalization()

            self.conv1_padding = Padding(padding=(0,0,0,0,2,1,2,1))
            self.conv1_ = gluon.nn.Conv2D(channels=96,
                kernel_size=(11,11),
                strides=(4,4),
                use_bias=True)
            # conv1_, output shape: {[96,55,55]}

            self.pool1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool1_, output shape: {[96,27,27]}

            self.relu1_ = gluon.nn.Activation(activation='relu')
            self.conv2_1_padding = Padding(padding=(0,0,0,0,2,2,2,2))
            self.conv2_1_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(5,5),
                strides=(1,1),
                use_bias=True)
            # conv2_1_, output shape: {[128,27,27]}

            self.pool2_1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool2_1_, output shape: {[128,13,13]}

            self.relu2_1_ = gluon.nn.Activation(activation='relu')
            self.conv2_2_padding = Padding(padding=(0,0,0,0,2,2,2,2))
            self.conv2_2_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(5,5),
                strides=(1,1),
                use_bias=True)
            # conv2_2_, output shape: {[128,27,27]}

            self.pool2_2_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool2_2_, output shape: {[128,13,13]}

            self.relu2_2_ = gluon.nn.Activation(activation='relu')
            self.conv3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv3_ = gluon.nn.Conv2D(channels=384,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv3_, output shape: {[384,13,13]}

            self.relu3_ = gluon.nn.Activation(activation='relu')
            self.conv4_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_ = gluon.nn.Conv2D(channels=192,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_, output shape: {[192,13,13]}

            self.relu4_1_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_1_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_1_, output shape: {[128,13,13]}

            self.pool5_1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool5_1_, output shape: {[128,6,6]}

            self.relu5_1_ = gluon.nn.Activation(activation='relu')
            self.conv4_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_2_ = gluon.nn.Conv2D(channels=192,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_2_, output shape: {[192,13,13]}

            self.relu4_2_ = gluon.nn.Activation(activation='relu')
            self.conv5_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_2_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_2_, output shape: {[128,13,13]}

            self.pool5_2_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool5_2_, output shape: {[128,6,6]}

            self.relu5_2_ = gluon.nn.Activation(activation='relu')
            self.fc6_ = gluon.nn.Dense(units=4096, use_bias=True, flatten=True)
            # fc6_, output shape: {[4096,1,1]}

            self.relu6_ = gluon.nn.Activation(activation='relu')
            self.dropout6_ = gluon.nn.Dropout(rate=0.5)
            self.fc7_ = gluon.nn.Dense(units=4096, use_bias=True, flatten=True)
            # fc7_, output shape: {[4096,1,1]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.dropout7_ = gluon.nn.Dropout(rate=0.5)
            self.fc8_ = gluon.nn.Dense(units=10, use_bias=True, flatten=True)
            # fc8_, output shape: {[10,1,1]}


            pass

    def hybrid_forward(self, F, data_):
        data_ = self.input_normalization_data_(data_)
        conv1_padding = self.conv1_padding(data_)
        conv1_ = self.conv1_(conv1_padding)
        lrn1_ = F.LRN(data=conv1_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5)
        pool1_ = self.pool1_(lrn1_)
        relu1_ = self.relu1_(pool1_)
        split1_ = F.split(relu1_, axis=1, num_outputs=2)
        get2_1_ = split1_[0]
        conv2_1_padding = self.conv2_1_padding(get2_1_)
        conv2_1_ = self.conv2_1_(conv2_1_padding)
        lrn2_1_ = F.LRN(data=conv2_1_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5)
        pool2_1_ = self.pool2_1_(lrn2_1_)
        relu2_1_ = self.relu2_1_(pool2_1_)
        get2_2_ = split1_[1]
        conv2_2_padding = self.conv2_2_padding(get2_2_)
        conv2_2_ = self.conv2_2_(conv2_2_padding)
        lrn2_2_ = F.LRN(data=conv2_2_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5)
        pool2_2_ = self.pool2_2_(lrn2_2_)
        relu2_2_ = self.relu2_2_(pool2_2_)
        concatenate3_ = F.concat(relu2_1_, relu2_2_, dim=1)
        conv3_padding = self.conv3_padding(concatenate3_)
        conv3_ = self.conv3_(conv3_padding)
        relu3_ = self.relu3_(conv3_)
        split3_ = F.split(relu3_, axis=1, num_outputs=2)
        get4_1_ = split3_[0]
        conv4_1_padding = self.conv4_1_padding(get4_1_)
        conv4_1_ = self.conv4_1_(conv4_1_padding)
        relu4_1_ = self.relu4_1_(conv4_1_)
        conv5_1_padding = self.conv5_1_padding(relu4_1_)
        conv5_1_ = self.conv5_1_(conv5_1_padding)
        pool5_1_ = self.pool5_1_(conv5_1_)
        relu5_1_ = self.relu5_1_(pool5_1_)
        get4_2_ = split3_[1]
        conv4_2_padding = self.conv4_2_padding(get4_2_)
        conv4_2_ = self.conv4_2_(conv4_2_padding)
        relu4_2_ = self.relu4_2_(conv4_2_)
        conv5_2_padding = self.conv5_2_padding(relu4_2_)
        conv5_2_ = self.conv5_2_(conv5_2_padding)
        pool5_2_ = self.pool5_2_(conv5_2_)
        relu5_2_ = self.relu5_2_(pool5_2_)
        concatenate6_ = F.concat(relu5_1_, relu5_2_, dim=1)
        fc6_ = self.fc6_(concatenate6_)
        relu6_ = self.relu6_(fc6_)
        dropout6_ = self.dropout6_(relu6_)
        fc7_ = self.fc7_(dropout6_)
        relu7_ = self.relu7_(fc7_)
        dropout7_ = self.dropout7_(relu7_)
        fc8_ = self.fc8_(dropout7_)
        softmax8_ = F.softmax(fc8_, axis=-1)
        predictions_ = softmax8_

        return predictions_

