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

            if data_mean:
                assert(data_std)
                self.input_normalization_action = ZScoreNormalization(data_mean=data_mean['action'],
                                                                               data_std=data_std['action'])
            else:
                self.input_normalization_action = NoNormalization()

            self.concatenate3_ = Concatenate(dim=1)
            # concatenate3_, output shape: {[32,1,1]}

            self.fc3_ = gluon.nn.Dense(units=400, use_bias=True)
            # fc3_, output shape: {[400,1,1]}

            self.relu3_ = gluon.nn.Activation(activation='relu')
            self.fc4_ = gluon.nn.Dense(units=300, use_bias=True)
            # fc4_, output shape: {[300,1,1]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.fc5_ = gluon.nn.Dense(units=1, use_bias=True)
            # fc5_, output shape: {[1,1,1]}



    def hybrid_forward(self, F, state, action):
        outputs = []
        state = self.input_normalization_state(state)
        action = self.input_normalization_action(action)
        concatenate3_ = self.concatenate3_(state, action)
        fc3_ = self.fc3_(concatenate3_)
        relu3_ = self.relu3_(fc3_)
        fc4_ = self.fc4_(relu3_)
        relu4_ = self.relu4_(fc4_)
        fc5_ = self.fc5_(relu4_)
        outputs.append(fc5_)

        return outputs[0]
