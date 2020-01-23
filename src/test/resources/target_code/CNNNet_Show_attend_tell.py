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

            pass

    def hybrid_forward(self, F, const1_):
        target_0_ = F.identity(const1_)

        return target_0_
class Net_1(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_1, self).__init__(**kwargs)
        with self.name_scope():
            if data_mean:
                assert(data_std)
                self.input_normalization_images_ = ZScoreNormalization(data_mean=data_mean['images_'],
                                                                               data_std=data_std['images_'])
            else:
                self.input_normalization_images_ = NoNormalization()

            self.conv1_padding = Padding(padding=(0,-1,0,0,0,0,0,0))
            self.conv1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(7,7),
                strides=(7,7),
                use_bias=True)
            # conv1_, output shape: {[64,32,32]}

            self.conv2_padding = Padding(padding=(0,-1,0,0,0,0,0,0))
            self.conv2_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(4,4),
                strides=(4,4),
                use_bias=True)
            # conv2_, output shape: {[64,8,8]}

            self.globalpooling2_ = gluon.nn.GlobalMaxPool2D()
            # globalpooling2_, output shape: {[64,1,1]}

            self.features_output_ = gluon.nn.Dense(units=256, use_bias=True, flatten=True)
            # features_output_, output shape: {}


            pass

    def hybrid_forward(self, F, images_):
        images_ = self.input_normalization_images_(images_)
        conv1_padding = self.conv1_padding(images_)
        conv1_ = self.conv1_(conv1_padding)
        conv2_padding = self.conv2_padding(conv1_)
        conv2_ = self.conv2_(conv2_padding)
        globalpooling2_ = self.globalpooling2_(conv2_)
        features_output_ = self.features_output_(globalpooling2_)

        return features_output_
class Net_2(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_2, self).__init__(**kwargs)
        with self.name_scope():
            self.fc5_1_1_1_ = gluon.nn.Dense(units=512, use_bias=True, flatten=False)
            # fc5_1_1_1_, output shape: {[512,1,1]}

            self.fc5_1_1_2_ = gluon.nn.Dense(units=512, use_bias=True, flatten=False)
            # fc5_1_1_2_, output shape: {[1,512,1]}

            self.tanh6_1_1_ = gluon.nn.Activation(activation='tanh')
            self.fc6_1_1_ = gluon.nn.Dense(units=1, use_bias=True, flatten=False)
            # fc6_1_1_, output shape: {[512,1,1]}

            self.dropout6_1_1_ = gluon.nn.Dropout(rate=0.25)
            self.attention_output_ = gluon.nn.Dense(units=1, use_bias=True, flatten=False)
            # attention_output_, output shape: {[1,1,1]}

            self.embedding3_2_ = gluon.nn.Embedding(input_dim=37759, output_dim=256)
            # embedding3_2_, output shape: {[1,256,1]}

            self.decoder_output_ = CustomLSTM(hidden_size=512,
                num_layers=1,
                bidirectional=False)
            # decoder_output_, output shape: {[1,512,1]}


            self.fc8_ = gluon.nn.Dense(units=37758, use_bias=True, flatten=True)
            # fc8_, output shape: {[37758,1,1]}

            self.tanh8_ = gluon.nn.Activation(activation='tanh')
            self.dropout8_ = gluon.nn.Dropout(rate=0.25)

            pass

    def hybrid_forward(self, F, features_output_, decoder_state_0_, target_999999_, decoder_state_1_):
        fc5_1_1_1_ = self.fc5_1_1_1_(features_output_)
        fc5_1_1_2_ = self.fc5_1_1_2_(decoder_state_0_)
        broadcastadd6_1_1_ = F.broadcast_add(fc5_1_1_1_,fc5_1_1_2_)
        tanh6_1_1_ = self.tanh6_1_1_(broadcastadd6_1_1_)
        fc6_1_1_ = self.fc6_1_1_(tanh6_1_1_)
        softmax6_1_1_ = F.softmax(fc6_1_1_, axis=1)
        dropout6_1_1_ = self.dropout6_1_1_(softmax6_1_1_)
        attention_output_ = self.attention_output_(dropout6_1_1_)
        broadcastmultiply7_1_ = F.broadcast_mul(attention_output_, features_output_)
        reducesum7_1_ = F.sum(broadcastmultiply7_1_, axis=1)
        expanddims7_1_ = F.expand_dims(reducesum7_1_, axis=1)
        embedding3_2_ = self.embedding3_2_(target_999999_)
        concatenate8_ = F.concat(expanddims7_1_, embedding3_2_, dim=2)
        decoder_output_, decoder_state_0_, decoder_state_1_ = self.decoder_output_(concatenate8_, decoder_state_0_, decoder_state_1_)

        fc8_ = self.fc8_(decoder_output_)
        tanh8_ = self.tanh8_(fc8_)
        dropout8_ = self.dropout8_(tanh8_)
        softmax8_ = F.softmax(dropout8_, axis=-1)
        argmax8_ = softmax8_
        target_1000000_ = F.identity(argmax8_)

        return target_1000000_, decoder_state_0_, decoder_state_1_, decoder_output_

    def getInputs(self):
        inputs = {}
        input_dimensions = (3,224,224)
        input_domains = (int,0.0,255.0)
        inputs["images_"] = input_domains + (input_dimensions,)
        return inputs

    def getOutputs(self):
        outputs = {}
        output_dimensions = ()
        output_domains = (int,0.0,0.0)
        outputs["target_0_"] = output_domains + (output_dimensions,)
        output_dimensions = (64,1,1)
        output_domains = (float,float('-inf'),float('inf'))
        outputs["features_output_"] = output_domains + (output_dimensions,)
        return outputs
