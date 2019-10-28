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
    def __init__(self, axis=-1, **kwargs):
        super(Softmax, self).__init__(**kwargs)
        with self.name_scope():
            self.axis = axis

    def hybrid_forward(self, F, x):
        return F.softmax(data=x, axis=self.axis)


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

class BroadcastMultiply(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(BroadcastMultiply, self).__init__(**kwargs)

    def hybrid_forward(self, F, *x):
        return F.broadcast_mul(*x)

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

class BroadcastAdd(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(BroadcastAdd, self).__init__(**kwargs)

    def hybrid_forward(self, F, *x):
        return F.broadcast_add(*x)


class Reshape(gluon.HybridBlock):
    def __init__(self, shape, **kwargs):
        super(Reshape, self).__init__(**kwargs)
        with self.name_scope():
            self.shape = shape

    def hybrid_forward(self, F, x):
        return F.reshape(data=x, shape=self.shape)

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


<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.isTrainable()>
class Net_${networkInstruction?index}(gluon.HybridBlock):
    def __init__(self, data_mean=None, data_std=None, **kwargs):
        super(Net_${networkInstruction?index}, self).__init__(**kwargs)
        self.last_layers = {}
        with self.name_scope():
${tc.include(networkInstruction.body, "ARCHITECTURE_DEFINITION")}

    def hybrid_forward(self, F, ${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ")}):
${tc.include(networkInstruction.body, "FORWARD_FUNCTION")}
        return ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ")}

</#if>
</#list>
