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

class Repeat(gluon.HybridBlock):
    def __init__(self, repeats, axis=1, **kwargs):
        super(Repeat, self).__init__(**kwargs)
        with self.name_scope():
            self.axis = axis
            self.repeats = repeats

    def hybrid_forward(self, F, x):
        return F.repeat(data=x, axis=self.axis, repeats=self.repeats)

class Dot(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(Dot, self).__init__(**kwargs)

    def hybrid_forward(self, F, *x):
        return F.batch_dot(*x)

class ExpandDims(gluon.HybridBlock):
    def __init__(self, dim=1, **kwargs):
        super(ExpandDims, self).__init__(**kwargs)
        with self.name_scope():
            self.dim = dim

    def hybrid_forward(self, F, x):
        return F.expand_dims(data=x, axis=self.dim)

class SwapAxes(gluon.HybridBlock):
    def __init__(self, dim1, dim2, **kwargs):
        super(SwapAxes, self).__init__(**kwargs)
        with self.name_scope():
            self.dim1 = dim1
            self.dim2 = dim2

    def hybrid_forward(self, F, x):
        return F.swapaxes(data=x, dim1=self.dim1, dim2=self.dim2)

class ReduceSum(gluon.HybridBlock):
    def __init__(self, axis=1, **kwargs):
        super(ReduceSum, self).__init__(**kwargs)
        with self.name_scope():
            self.axis = axis

    def hybrid_forward(self, F, x):
        return F.sum(data=x, axis=self.axis)

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
                self.input_normalization_state_ = ZScoreNormalization(data_mean=data_mean['state_'],
                                                                               data_std=data_std['state_'])
            else:
                self.input_normalization_state_ = NoNormalization()

            self.fc2_1_ = gluon.nn.Dense(units=300, use_bias=True, flatten=True)
            # fc2_1_, output shape: {[300,1,1]}

            self.relu2_1_ = gluon.nn.Activation(activation='relu')
            self.fc3_1_ = gluon.nn.Dense(units=600, use_bias=True, flatten=True)
            # fc3_1_, output shape: {[600,1,1]}

            if data_mean:
                assert(data_std)
                self.input_normalization_action_ = ZScoreNormalization(data_mean=data_mean['action_'],
                                                                               data_std=data_std['action_'])
            else:
                self.input_normalization_action_ = NoNormalization()

            self.fc2_2_ = gluon.nn.Dense(units=600, use_bias=True, flatten=True)
            # fc2_2_, output shape: {[600,1,1]}

            self.fc4_ = gluon.nn.Dense(units=600, use_bias=True, flatten=True)
            # fc4_, output shape: {[600,1,1]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.fc5_ = gluon.nn.Dense(units=1, use_bias=True, flatten=True)
            # fc5_, output shape: {[1,1,1]}



    def hybrid_forward(self, F, state_, action_):
        state_ = self.input_normalization_state_(state_)
        fc2_1_ = self.fc2_1_(state_)
        relu2_1_ = self.relu2_1_(fc2_1_)
        fc3_1_ = self.fc3_1_(relu2_1_)
        action_ = self.input_normalization_action_(action_)
        fc2_2_ = self.fc2_2_(action_)
        add4_ = fc3_1_ + fc2_2_
        fc4_ = self.fc4_(add4_)
        relu4_ = self.relu4_(fc4_)
        fc5_ = self.fc5_(relu4_)
        qvalues_ = fc5_

        return qvalues_

