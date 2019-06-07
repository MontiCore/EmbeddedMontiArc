import mxnet as mx
import numpy as np
from mxnet import gluon

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


class Net(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(tc.architecture.body, "ARCHITECTURE_DEFINITION")}

    def hybrid_forward(self, F, <#list tc.architecture.inputs as input>${input}<#if input?has_next>, </#if></#list>):
${tc.include(tc.architecture.body, "FORWARD_FUNCTION")}