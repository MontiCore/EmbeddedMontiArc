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
                self.input_normalization_source_ = ZScoreNormalization(data_mean=data_mean['source_'],
                                                                               data_std=data_std['source_'])
            else:
                self.input_normalization_source_ = NoNormalization()

            self.embedding1_ = gluon.nn.Embedding(input_dim=50000, output_dim=620)
            # embedding1_, output shape: {[30,620,1]}

            self.encoder_output_ = CustomGRU(hidden_size=1000,
                num_layers=1,
                bidirectional=True)
            # encoder_output_, output shape: {[30,2000,1]}

            self.fc_output_ = gluon.nn.Dense(units=1000, use_bias=True, flatten=False)
            # fc_output_, output shape: {}


            pass

    def hybrid_forward(self, F, source_, encoder_state_):
        source_ = self.input_normalization_source_(source_)
        embedding1_ = self.embedding1_(source_)
        encoder_output_, encoder_state_ = self.encoder_output_(embedding1_, encoder_state_)
        fc_output_ = self.fc_output_(encoder_output_)

        return fc_output_, encoder_state_, encoder_output_
class Net_1(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_1, self).__init__(**kwargs)
        with self.name_scope():

            pass

    def hybrid_forward(self, F, const1_):
        target_0_ = F.identity(const1_)

        return target_0_
class Net_2(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_2, self).__init__(**kwargs)
        with self.name_scope():


            pass

    def hybrid_forward(self, F, encoder_state_):

        split1_ = F.split(encoder_state_, axis=1, num_outputs=2)
        get1_ = split1_[1]
        decoder_state_ = F.identity(get1_)

        return decoder_state_
class Net_3(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_3, self).__init__(**kwargs)
        with self.name_scope():
            self.fc5_1_1_ = gluon.nn.Dense(units=1000, use_bias=True, flatten=False)
            # fc5_1_1_, output shape: {[30,1000,1]}

            self.tanh5_1_1_ = gluon.nn.Activation(activation='tanh')
            self.fc6_1_1_ = gluon.nn.Dense(units=30, use_bias=True, flatten=True)
            # fc6_1_1_, output shape: {[30,1,1]}

            self.embedding2_2_ = gluon.nn.Embedding(input_dim=50000, output_dim=620)
            # embedding2_2_, output shape: {[1,620,1]}

            self.decoder_output_ = CustomGRU(hidden_size=1000,
                num_layers=1,
                bidirectional=False)
            # decoder_output_, output shape: {[1,1000,1]}

            self.fc8_ = gluon.nn.Dense(units=50000, use_bias=True, flatten=True)
            # fc8_, output shape: {[50000,1,1]}


            pass

    def hybrid_forward(self, F, decoder_state_, fc_output_, target_999999_):
        repeat4_1_1_1_ = F.repeat(decoder_state_, repeats=30, axis=1)
        concatenate5_1_1_ = F.concat(repeat4_1_1_1_, fc_output_, dim=2)
        fc5_1_1_ = self.fc5_1_1_(concatenate5_1_1_)
        tanh5_1_1_ = self.tanh5_1_1_(fc5_1_1_)
        fc6_1_1_ = self.fc6_1_1_(tanh5_1_1_)
        softmax6_1_1_ = F.softmax(fc6_1_1_, axis=-1)
        expanddims6_1_1_ = F.expand_dims(softmax6_1_1_, axis=1)
        dot7_1_ = F.batch_dot(expanddims6_1_1_, fc_output_)
        embedding2_2_ = self.embedding2_2_(target_999999_)
        concatenate8_ = F.concat(dot7_1_, embedding2_2_, dim=2)
        decoder_output_, decoder_state_ = self.decoder_output_(concatenate8_, decoder_state_)
        fc8_ = self.fc8_(decoder_output_)
        softmax8_ = F.softmax(fc8_, axis=-1)
        argmax8_ = softmax8_
        target_1000000_ = F.identity(argmax8_)

        return target_1000000_, decoder_state_, decoder_output_

    def getInputs(self):
        inputs = {}
        input_dimensions = (30)
        input_domains = (int,0.0,49999.0)
        inputs["source_"] = input_domains + (input_dimensions,)
        input_dimensions = (2,1000,1)
        input_domains = (float,float('-inf'),float('inf'))
        inputs["encoder_state_"] = input_domains + (input_dimensions,)
        return inputs

    def getOutputs(self):
        outputs = {}
        output_dimensions = (30,2000,1)
        output_domains = (float,float('-inf'),float('inf'))
        outputs["fc_output_"] = output_domains + (output_dimensions,)
        output_dimensions = ()
        output_domains = (int,1.0,1.0)
        outputs["target_0_"] = output_domains + (output_dimensions,)
        output_dimensions = (1,1000,1)
        output_domains = (float,float('-inf'),float('inf'))
        outputs["decoder_state_"] = output_domains + (output_dimensions,)
        return outputs
