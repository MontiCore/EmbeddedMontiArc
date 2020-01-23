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

            self.fc1_ = gluon.nn.Dense(units=4, use_bias=True, flatten=True)
            # fc1_, output shape: {[4,1,1]}


            pass

    def hybrid_forward(self, F, data_0_):
        data_0_ = self.input_normalization_data_0_(data_0_)
        fc1_ = self.fc1_(data_0_)
        softmax1_ = F.softmax(fc1_, axis=-1)
        pred_0_ = F.identity(softmax1_)

        return pred_0_
class Net_1(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_1, self).__init__(**kwargs)
        with self.name_scope():
            if data_mean:
                assert(data_std)
                self.input_normalization_data_1_ = ZScoreNormalization(data_mean=data_mean['data_1_'],
                                                                               data_std=data_std['data_1_'])
            else:
                self.input_normalization_data_1_ = NoNormalization()



            pass

    def hybrid_forward(self, F, data_1_):
        data_1_ = self.input_normalization_data_1_(data_1_)
        onehot1_ = F.one_hot(indices=data_1_, depth=4)

        pred_1_ = F.identity(onehot1_)

        return pred_1_
class Net_2(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_2, self).__init__(**kwargs)
        with self.name_scope():


            pass

    def hybrid_forward(self, F, const1_):
        onehot2_ = F.one_hot(indices=const1_, depth=4)

        pred_2_ = F.identity(onehot2_)

        return pred_2_

    def getInputs(self):
        inputs = {}
        input_dimensions = (1)
        input_domains = (int,0.0,3.0)
        inputs["data_0_"] = input_domains + (input_dimensions,)
        input_dimensions = (1)
        input_domains = (int,0.0,3.0)
        inputs["data_1_"] = input_domains + (input_dimensions,)
        return inputs

    def getOutputs(self):
        outputs = {}
        output_dimensions = (4,1,1)
        output_domains = (float,0.0,1.0)
        outputs["pred_0_"] = output_domains + (output_dimensions,)
        output_dimensions = (4,1,1)
        output_domains = (float,0.0,1.0)
        outputs["pred_1_"] = output_domains + (output_dimensions,)
        output_dimensions = (4,1,1)
        output_domains = (float,0.0,1.0)
        outputs["pred_2_"] = output_domains + (output_dimensions,)
        return outputs
