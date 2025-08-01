import mxnet as mx
import numpy as np
from mxnet import gluon

class OneHot(gluon.HybridBlock):
    def __init__(self, size, **kwargs):
        super(OneHot, self).__init__(**kwargs)
        with self.name_scope():
            self.size = size

    def hybrid_forward(self, F, x):
        return F.one_hot(indices=F.argmax(data=x, axis=1), depth=self.size)


class Softmax(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(Softmax, self).__init__(**kwargs)

    def hybrid_forward(self, F, x):
        return F.softmax(x)


class Split(gluon.HybridBlock):
    def __init__(self, num_outputs, axis=1, **kwargs):
        super(Split, self).__init__(**kwargs)
        with self.name_scope():
            self.axis = axis
            self.num_outputs = num_outputs

    def hybrid_forward(self, F, x):
        return F.split(data=x, axis=self.axis, num_outputs=self.num_outputs)


class Concatenate(gluon.HybridBlock):
    def __init__(self, dim=1, **kwargs):
        super(Concatenate, self).__init__(**kwargs)
        with self.name_scope():
            self.dim = dim

    def hybrid_forward(self, F, *x):
        return F.concat(*x, dim=self.dim)


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


class Net_0(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_0, self).__init__(**kwargs)
        self.last_layers = {}
        with self.name_scope():
            if data_mean:
                assert(data_std)
                self.input_normalization_state = ZScoreNormalization(data_mean=data_mean['state'],
                                                                               data_std=data_std['state'])
            else:
                self.input_normalization_state = NoNormalization()

            self.fc1_ = gluon.nn.Dense(units=400, use_bias=True)
            # fc1_, output shape: {[400,1,1]}

            self.relu1_ = gluon.nn.Activation(activation='relu')
            self.fc2_ = gluon.nn.Dense(units=300, use_bias=True)
            # fc2_, output shape: {[300,1,1]}

            self.relu2_ = gluon.nn.Activation(activation='relu')
            self.fc3_ = gluon.nn.Dense(units=6, use_bias=True)
            # fc3_, output shape: {[6,1,1]}

            self.tanh3_ = gluon.nn.Activation(activation='tanh')


    def hybrid_forward(self, F, state):
        outputs = []
        state = self.input_normalization_state(state)
        fc1_ = self.fc1_(state)
        relu1_ = self.relu1_(fc1_)
        fc2_ = self.fc2_(relu1_)
        relu2_ = self.relu2_(fc2_)
        fc3_ = self.fc3_(relu2_)
        tanh3_ = self.tanh3_(fc3_)
        outputs.append(tanh3_)

        return outputs[0]
