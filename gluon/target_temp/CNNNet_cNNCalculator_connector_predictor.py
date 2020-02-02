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
                self.input_normalization_noise_ = ZScoreNormalization(data_mean=data_mean['noise_'],
                                                                               data_std=data_std['noise_'])
            else:
                self.input_normalization_noise_ = NoNormalization()

            self.transconv1_padding = (0,0)
            self.transconv1_ = gluon.nn.Conv2DTranspose(channels=1024,
                kernel_size=(4,4),
                strides=(1,1),
                padding=self.transconv1_padding,
                use_bias=False)
            # transconv1_, output shape: {[1024,4,4]}

            self.batchnorm1_ = gluon.nn.BatchNorm()
            # batchnorm1_, output shape: {[1024,4,4]}

            self.relu1_ = gluon.nn.Activation(activation='relu')
            self.transconv2_padding = (1,1)
            self.transconv2_ = gluon.nn.Conv2DTranspose(channels=512,
                kernel_size=(4,4),
                strides=(2,2),
                padding=self.transconv2_padding,
                use_bias=False)
            # transconv2_, output shape: {[512,8,8]}

            self.batchnorm2_ = gluon.nn.BatchNorm()
            # batchnorm2_, output shape: {[512,8,8]}

            self.relu2_ = gluon.nn.Activation(activation='relu')
            self.transconv3_padding = (1,1)
            self.transconv3_ = gluon.nn.Conv2DTranspose(channels=256,
                kernel_size=(4,4),
                strides=(2,2),
                padding=self.transconv3_padding,
                use_bias=False)
            # transconv3_, output shape: {[256,16,16]}

            self.batchnorm3_ = gluon.nn.BatchNorm()
            # batchnorm3_, output shape: {[256,16,16]}

            self.relu3_ = gluon.nn.Activation(activation='relu')
            self.transconv4_padding = (1,1)
            self.transconv4_ = gluon.nn.Conv2DTranspose(channels=128,
                kernel_size=(4,4),
                strides=(2,2),
                padding=self.transconv4_padding,
                use_bias=False)
            # transconv4_, output shape: {[128,32,32]}

            self.batchnorm4_ = gluon.nn.BatchNorm()
            # batchnorm4_, output shape: {[128,32,32]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.transconv5_padding = (1,1)
            self.transconv5_ = gluon.nn.Conv2DTranspose(channels=1,
                kernel_size=(4,4),
                strides=(2,2),
                padding=self.transconv5_padding,
                use_bias=False)
            # transconv5_, output shape: {[1,64,64]}

            self.tanh5_ = gluon.nn.Activation(activation='tanh')

            pass

    def hybrid_forward(self, F, noise_):
        noise_ = self.input_normalization_noise_(noise_)
        transconv1_ = self.transconv1_(noise_)
        batchnorm1_ = self.batchnorm1_(transconv1_)
        relu1_ = self.relu1_(batchnorm1_)
        transconv2_ = self.transconv2_(relu1_)
        batchnorm2_ = self.batchnorm2_(transconv2_)
        relu2_ = self.relu2_(batchnorm2_)
        transconv3_ = self.transconv3_(relu2_)
        batchnorm3_ = self.batchnorm3_(transconv3_)
        relu3_ = self.relu3_(batchnorm3_)
        transconv4_ = self.transconv4_(relu3_)
        batchnorm4_ = self.batchnorm4_(transconv4_)
        relu4_ = self.relu4_(batchnorm4_)
        transconv5_ = self.transconv5_(relu4_)
        tanh5_ = self.tanh5_(transconv5_)
        image_ = F.identity(tanh5_)

        return image_

    def getInputs(self):
        inputs = {}
        input_dimensions = (100,1,1)
        input_domains = (float,0.0,1.0)
        inputs["noise_"] = input_domains + (input_dimensions,)
        return inputs

    def getOutputs(self):
        outputs = {}
        output_dimensions = (1,64,64)
        output_domains = (float,-1.0,1.0)
        outputs["image_"] = output_domains + (output_dimensions,)
        return outputs
