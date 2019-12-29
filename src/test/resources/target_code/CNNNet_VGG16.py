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
                self.input_normalization_data_ = ZScoreNormalization(data_mean=data_mean['data_'],
                                                                               data_std=data_std['data_'])
            else:
                self.input_normalization_data_ = NoNormalization()

            self.conv1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv1_, output shape: {[64,224,224]}

            self.relu1_ = gluon.nn.Activation(activation='relu')
            self.conv2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv2_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv2_, output shape: {[64,224,224]}

            self.relu2_ = gluon.nn.Activation(activation='relu')
            self.pool2_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool2_, output shape: {[64,112,112]}

            self.conv3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv3_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv3_, output shape: {[128,112,112]}

            self.relu3_ = gluon.nn.Activation(activation='relu')
            self.conv4_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_, output shape: {[128,112,112]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.pool4_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool4_, output shape: {[128,56,56]}

            self.conv5_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_, output shape: {[256,56,56]}

            self.relu5_ = gluon.nn.Activation(activation='relu')
            self.conv6_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv6_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv6_, output shape: {[256,56,56]}

            self.relu6_ = gluon.nn.Activation(activation='relu')
            self.conv7_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv7_ = gluon.nn.Conv2D(channels=256,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv7_, output shape: {[256,56,56]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.pool7_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool7_, output shape: {[256,28,28]}

            self.conv8_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv8_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv8_, output shape: {[512,28,28]}

            self.relu8_ = gluon.nn.Activation(activation='relu')
            self.conv9_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv9_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv9_, output shape: {[512,28,28]}

            self.relu9_ = gluon.nn.Activation(activation='relu')
            self.conv10_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv10_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv10_, output shape: {[512,28,28]}

            self.relu10_ = gluon.nn.Activation(activation='relu')
            self.pool10_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool10_, output shape: {[512,14,14]}

            self.conv11_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv11_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv11_, output shape: {[512,14,14]}

            self.relu11_ = gluon.nn.Activation(activation='relu')
            self.conv12_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv12_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv12_, output shape: {[512,14,14]}

            self.relu12_ = gluon.nn.Activation(activation='relu')
            self.conv13_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv13_ = gluon.nn.Conv2D(channels=512,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv13_, output shape: {[512,14,14]}

            self.relu13_ = gluon.nn.Activation(activation='relu')
            self.pool13_ = gluon.nn.MaxPool2D(
                pool_size=(2,2),
                strides=(2,2))
            # pool13_, output shape: {[512,7,7]}

            self.fc13_ = gluon.nn.Dense(units=4096, use_bias=True, flatten=True)
            # fc13_, output shape: {[4096,1,1]}

            self.relu14_ = gluon.nn.Activation(activation='relu')
            self.dropout14_ = gluon.nn.Dropout(rate=0.5)
            self.fc14_ = gluon.nn.Dense(units=4096, use_bias=True, flatten=True)
            # fc14_, output shape: {[4096,1,1]}

            self.relu15_ = gluon.nn.Activation(activation='relu')
            self.dropout15_ = gluon.nn.Dropout(rate=0.5)
            self.fc15_ = gluon.nn.Dense(units=1000, use_bias=True, flatten=True)
            # fc15_, output shape: {[1000,1,1]}


            pass

    def hybrid_forward(self, F, data_):
        data_ = self.input_normalization_data_(data_)
        conv1_padding = self.conv1_padding(data_)
        conv1_ = self.conv1_(conv1_padding)
        relu1_ = self.relu1_(conv1_)
        conv2_padding = self.conv2_padding(relu1_)
        conv2_ = self.conv2_(conv2_padding)
        relu2_ = self.relu2_(conv2_)
        pool2_ = self.pool2_(relu2_)
        conv3_padding = self.conv3_padding(pool2_)
        conv3_ = self.conv3_(conv3_padding)
        relu3_ = self.relu3_(conv3_)
        conv4_padding = self.conv4_padding(relu3_)
        conv4_ = self.conv4_(conv4_padding)
        relu4_ = self.relu4_(conv4_)
        pool4_ = self.pool4_(relu4_)
        conv5_padding = self.conv5_padding(pool4_)
        conv5_ = self.conv5_(conv5_padding)
        relu5_ = self.relu5_(conv5_)
        conv6_padding = self.conv6_padding(relu5_)
        conv6_ = self.conv6_(conv6_padding)
        relu6_ = self.relu6_(conv6_)
        conv7_padding = self.conv7_padding(relu6_)
        conv7_ = self.conv7_(conv7_padding)
        relu7_ = self.relu7_(conv7_)
        pool7_ = self.pool7_(relu7_)
        conv8_padding = self.conv8_padding(pool7_)
        conv8_ = self.conv8_(conv8_padding)
        relu8_ = self.relu8_(conv8_)
        conv9_padding = self.conv9_padding(relu8_)
        conv9_ = self.conv9_(conv9_padding)
        relu9_ = self.relu9_(conv9_)
        conv10_padding = self.conv10_padding(relu9_)
        conv10_ = self.conv10_(conv10_padding)
        relu10_ = self.relu10_(conv10_)
        pool10_ = self.pool10_(relu10_)
        conv11_padding = self.conv11_padding(pool10_)
        conv11_ = self.conv11_(conv11_padding)
        relu11_ = self.relu11_(conv11_)
        conv12_padding = self.conv12_padding(relu11_)
        conv12_ = self.conv12_(conv12_padding)
        relu12_ = self.relu12_(conv12_)
        conv13_padding = self.conv13_padding(relu12_)
        conv13_ = self.conv13_(conv13_padding)
        relu13_ = self.relu13_(conv13_)
        pool13_ = self.pool13_(relu13_)
        fc13_ = self.fc13_(pool13_)
        relu14_ = self.relu14_(fc13_)
        dropout14_ = self.dropout14_(relu14_)
        fc14_ = self.fc14_(dropout14_)
        relu15_ = self.relu15_(fc14_)
        dropout15_ = self.dropout15_(relu15_)
        fc15_ = self.fc15_(dropout15_)
        softmax15_ = F.softmax(fc15_, axis=-1)
        predictions_ = F.identity(softmax15_)

        return predictions_

    def getInputs(self):
        inputs = {}
        input_dimensions = (3,224,224)
        input_domains = (int,0.0,255.0)
        inputs["data_"] = input_domains + (input_dimensions,)
        return inputs

    def getOutputs(self):
        outputs = {}
        output_dimensions = (1000,1,1)
        output_domains = (float,0.0,1.0)
        outputs["predictions_"] = output_domains + (output_dimensions,)
        return outputs
