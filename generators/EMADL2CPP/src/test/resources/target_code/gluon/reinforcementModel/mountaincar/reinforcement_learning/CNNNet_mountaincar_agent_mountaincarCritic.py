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
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomRNN, self).__init__(**kwargs)
        with self.name_scope():
            self.rnn = gluon.rnn.RNN(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                     bidirectional=bidirectional, activation='tanh', layout='NTC')

    def hybrid_forward(self, F, data, state0):
        output, [state0] = self.rnn(data, [F.swapaxes(state0, 0, 1)])
        return output, F.swapaxes(state0, 0, 1)


class CustomLSTM(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomLSTM, self).__init__(**kwargs)
        with self.name_scope():
            self.lstm = gluon.rnn.LSTM(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
                                       bidirectional=bidirectional, layout='NTC')

    def hybrid_forward(self, F, data, state0, state1):
        output, [state0, state1] = self.lstm(data, [F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)])
        return output, F.swapaxes(state0, 0, 1), F.swapaxes(state1, 0, 1)


class CustomGRU(gluon.HybridBlock):
    def __init__(self, hidden_size, num_layers, dropout, bidirectional, **kwargs):
        super(CustomGRU, self).__init__(**kwargs)
        with self.name_scope():
            self.gru = gluon.rnn.GRU(hidden_size=hidden_size, num_layers=num_layers, dropout=dropout,
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
                self.input_normalization_state_ = ZScoreNormalization(data_mean=data_mean['state_'],
                                                                               data_std=data_std['state_'])
            else:
                self.input_normalization_state_ = NoNormalization()

            self.fc2_1_ = gluon.nn.Dense(units=400, use_bias=True, flatten=True)
            # fc2_1_, output shape: {[400,1,1]}

            self.relu2_1_ = gluon.nn.Activation(activation='relu')
            self.fc3_1_ = gluon.nn.Dense(units=300, use_bias=True, flatten=True)
            # fc3_1_, output shape: {[300,1,1]}

            if data_mean:
                assert(data_std)
                self.input_normalization_action_ = ZScoreNormalization(data_mean=data_mean['action_'],
                                                                               data_std=data_std['action_'])
            else:
                self.input_normalization_action_ = NoNormalization()

            self.fc2_2_ = gluon.nn.Dense(units=300, use_bias=True, flatten=True)
            # fc2_2_, output shape: {[300,1,1]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.fc4_ = gluon.nn.Dense(units=1, use_bias=True, flatten=True)
            # fc4_, output shape: {[1,1,1]}


            pass

    def hybrid_forward(self, F, state_, action_):
        state_ = self.input_normalization_state_(state_)
        fc2_1_ = self.fc2_1_(state_)
        relu2_1_ = self.relu2_1_(fc2_1_)
        fc3_1_ = self.fc3_1_(relu2_1_)
        action_ = self.input_normalization_action_(action_)
        fc2_2_ = self.fc2_2_(action_)
        add4_ = fc3_1_ + fc2_2_
        relu4_ = self.relu4_(add4_)
        fc4_ = self.fc4_(relu4_)
        qvalues_ = F.identity(fc4_)

        return qvalues_
