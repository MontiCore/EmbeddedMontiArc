import mxnet as mx
import numpy as np
import math
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
                self.input_normalization_data_0_ = ZScoreNormalization(data_mean=data_mean['data_0_'],
                                                                               data_std=data_std['data_0_'])
            else:
                self.input_normalization_data_0_ = NoNormalization()

            if data_mean:
                assert(data_std)
                self.input_normalization_data_1_ = ZScoreNormalization(data_mean=data_mean['data_1_'],
                                                                               data_std=data_std['data_1_'])
            else:
                self.input_normalization_data_1_ = NoNormalization()

            if data_mean:
                assert(data_std)
                self.input_normalization_data_2_ = ZScoreNormalization(data_mean=data_mean['data_2_'],
                                                                               data_std=data_std['data_2_'])
            else:
                self.input_normalization_data_2_ = NoNormalization()

            self.conv4_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_, output shape: {[32,200,300]}

            self.relu4_1_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_1_, output shape: {[32,200,300]}

            self.relu5_1_ = gluon.nn.Activation(activation='relu')
            self.conv6_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv6_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv6_1_, output shape: {[32,200,300]}

            self.relu6_1_ = gluon.nn.Activation(activation='relu')
            self.pool6_1_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool6_1_, output shape: {[32,100,150]}

            self.conv4_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_2_, output shape: {[32,200,300]}

            self.relu4_2_ = gluon.nn.Activation(activation='relu')
            self.conv5_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_2_, output shape: {[32,200,300]}

            self.relu5_2_ = gluon.nn.Activation(activation='relu')
            self.conv6_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv6_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv6_2_, output shape: {[32,200,300]}

            self.relu6_2_ = gluon.nn.Activation(activation='relu')
            self.pool6_2_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool6_2_, output shape: {[32,100,150]}

            self.conv4_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_3_, output shape: {[32,200,300]}

            self.relu4_3_ = gluon.nn.Activation(activation='relu')
            self.conv5_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_3_, output shape: {[32,200,300]}

            self.relu5_3_ = gluon.nn.Activation(activation='relu')
            self.conv6_3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv6_3_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv6_3_, output shape: {[32,200,300]}

            self.relu6_3_ = gluon.nn.Activation(activation='relu')
            self.pool6_3_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool6_3_, output shape: {[32,100,150]}

            self.conv7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv7_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv7_, output shape: {[64,100,150]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.pool7_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool7_, output shape: {[64,50,75]}

            self.fc7_ = gluon.nn.Dense(units=32, use_bias=True, flatten=True)
            # fc7_, output shape: {[32,1,1]}

            self.relu8_ = gluon.nn.Activation(activation='relu')
            self.fc8_ = gluon.nn.Dense(units=3, use_bias=True, flatten=True)
            # fc8_, output shape: {[3,1,1]}


            pass

    def hybrid_forward(self, F, data_0_, data_1_, data_2_):
        data_0_ = self.input_normalization_data_0_(data_0_)
        data_1_ = self.input_normalization_data_1_(data_1_)
        data_2_ = self.input_normalization_data_2_(data_2_)
        get4_1_ = data_0_
        conv4_1_padding = self.conv4_1_padding(get4_1_)
        conv4_1_ = self.conv4_1_(conv4_1_padding)
        relu4_1_ = self.relu4_1_(conv4_1_)
        conv5_1_padding = self.conv5_1_padding(relu4_1_)
        conv5_1_ = self.conv5_1_(conv5_1_padding)
        relu5_1_ = self.relu5_1_(conv5_1_)
        conv6_1_padding = self.conv6_1_padding(relu5_1_)
        conv6_1_ = self.conv6_1_(conv6_1_padding)
        relu6_1_ = self.relu6_1_(conv6_1_)
        pool6_1_ = self.pool6_1_(relu6_1_)
        get4_2_ = data_1_
        conv4_2_padding = self.conv4_2_padding(get4_2_)
        conv4_2_ = self.conv4_2_(conv4_2_padding)
        relu4_2_ = self.relu4_2_(conv4_2_)
        conv5_2_padding = self.conv5_2_padding(relu4_2_)
        conv5_2_ = self.conv5_2_(conv5_2_padding)
        relu5_2_ = self.relu5_2_(conv5_2_)
        conv6_2_padding = self.conv6_2_padding(relu5_2_)
        conv6_2_ = self.conv6_2_(conv6_2_padding)
        relu6_2_ = self.relu6_2_(conv6_2_)
        pool6_2_ = self.pool6_2_(relu6_2_)
        get4_3_ = data_2_
        conv4_3_padding = self.conv4_3_padding(get4_3_)
        conv4_3_ = self.conv4_3_(conv4_3_padding)
        relu4_3_ = self.relu4_3_(conv4_3_)
        conv5_3_padding = self.conv5_3_padding(relu4_3_)
        conv5_3_ = self.conv5_3_(conv5_3_padding)
        relu5_3_ = self.relu5_3_(conv5_3_)
        conv6_3_padding = self.conv6_3_padding(relu5_3_)
        conv6_3_ = self.conv6_3_(conv6_3_padding)
        relu6_3_ = self.relu6_3_(conv6_3_)
        pool6_3_ = self.pool6_3_(relu6_3_)
        concatenate7_ = F.concat(pool6_1_, pool6_2_, pool6_3_, dim=1)
        conv7_padding = self.conv7_padding(concatenate7_)
        conv7_ = self.conv7_(conv7_padding)
        relu7_ = self.relu7_(conv7_)
        pool7_ = self.pool7_(relu7_)
        fc7_ = self.fc7_(pool7_)
        relu8_ = self.relu8_(fc7_)
        fc8_ = self.fc8_(relu8_)
        softmax8_ = F.softmax(fc8_, axis=-1)
        predictions_ = F.identity(softmax8_)

        return predictions_

    def getInputs(self):
        inputs = {}
        input_dimensions = (3,200,300)
        input_domains = (int,0.0,255.0)
        inputs["data_0_"] = input_domains + (input_dimensions,)
        input_dimensions = (3,200,300)
        input_domains = (int,0.0,255.0)
        inputs["data_1_"] = input_domains + (input_dimensions,)
        input_dimensions = (3,200,300)
        input_domains = (int,0.0,255.0)
        inputs["data_2_"] = input_domains + (input_dimensions,)
        return inputs

    def getOutputs(self):
        outputs = {}
        output_dimensions = (3,1,1)
        output_domains = (float,0.0,1.0)
        outputs["predictions_"] = output_domains + (output_dimensions,)
        return outputs
