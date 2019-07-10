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
                self.input_normalization_data = ZScoreNormalization(data_mean=data_mean['data'],
                                                                               data_std=data_std['data'])
            else:
                self.input_normalization_data = NoNormalization()

            self.conv1_padding = Padding(padding=(0,0,0,0,2,1,2,1))
            self.conv1_ = gluon.nn.Conv2D(channels=96,
                kernel_size=(11,11),
                strides=(4,4),
                use_bias=True)
            # conv1_, output shape: {[96,55,55]}

            self.pool1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool1_, output shape: {[96,27,27]}

            self.relu1_ = gluon.nn.Activation(activation='relu')
            self.split1_ = Split(num_outputs=2, axis=1)
            # split1_, output shape: {[48,27,27][48,27,27]}

            self.conv2_1_padding = Padding(padding=(0,0,0,0,2,2,2,2))
            self.conv2_1_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(5,5),
                strides=(1,1),
                use_bias=True)
            # conv2_1_, output shape: {[128,27,27]}

            self.pool2_1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool2_1_, output shape: {[128,13,13]}

            self.relu2_1_ = gluon.nn.Activation(activation='relu')
            self.conv2_2_padding = Padding(padding=(0,0,0,0,2,2,2,2))
            self.conv2_2_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(5,5),
                strides=(1,1),
                use_bias=True)
            # conv2_2_, output shape: {[128,27,27]}

            self.pool2_2_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool2_2_, output shape: {[128,13,13]}

            self.relu2_2_ = gluon.nn.Activation(activation='relu')
            self.concatenate3_ = Concatenate(dim=1)
            # concatenate3_, output shape: {[256,13,13]}

            self.conv3_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv3_ = gluon.nn.Conv2D(channels=384,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv3_, output shape: {[384,13,13]}

            self.relu3_ = gluon.nn.Activation(activation='relu')
            self.split3_ = Split(num_outputs=2, axis=1)
            # split3_, output shape: {[192,13,13][192,13,13]}

            self.conv4_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_1_ = gluon.nn.Conv2D(channels=192,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_1_, output shape: {[192,13,13]}

            self.relu4_1_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_1_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_1_, output shape: {[128,13,13]}

            self.pool5_1_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool5_1_, output shape: {[128,6,6]}

            self.relu5_1_ = gluon.nn.Activation(activation='relu')
            self.conv4_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv4_2_ = gluon.nn.Conv2D(channels=192,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv4_2_, output shape: {[192,13,13]}

            self.relu4_2_ = gluon.nn.Activation(activation='relu')
            self.conv5_2_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv5_2_ = gluon.nn.Conv2D(channels=128,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv5_2_, output shape: {[128,13,13]}

            self.pool5_2_ = gluon.nn.MaxPool2D(
                pool_size=(3,3),
                strides=(2,2))
            # pool5_2_, output shape: {[128,6,6]}

            self.relu5_2_ = gluon.nn.Activation(activation='relu')
            self.concatenate6_ = Concatenate(dim=1)
            # concatenate6_, output shape: {[256,6,6]}

            self.fc6_flatten = gluon.nn.Flatten()
            self.fc6_ = gluon.nn.Dense(units=4096, use_bias=True)
            # fc6_, output shape: {[4096,1,1]}

            self.relu6_ = gluon.nn.Activation(activation='relu')
            self.dropout6_ = gluon.nn.Dropout(rate=0.5)
            self.fc7_ = gluon.nn.Dense(units=4096, use_bias=True)
            # fc7_, output shape: {[4096,1,1]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.dropout7_ = gluon.nn.Dropout(rate=0.5)
            self.fc8_ = gluon.nn.Dense(units=10, use_bias=True)
            # fc8_, output shape: {[10,1,1]}

            self.softmax8_ = Softmax()


    def hybrid_forward(self, F, data):
        outputs = []
        data = self.input_normalization_data(data)
        conv1_padding = self.conv1_padding(data)
        conv1_ = self.conv1_(conv1_padding)
        lrn1_ = F.LRN(data=conv1_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5)
        pool1_ = self.pool1_(lrn1_)
        relu1_ = self.relu1_(pool1_)
        split1_ = self.split1_(relu1_)
        get2_1_ = split1_[0]
        conv2_1_padding = self.conv2_1_padding(get2_1_)
        conv2_1_ = self.conv2_1_(conv2_1_padding)
        lrn2_1_ = F.LRN(data=conv2_1_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5)
        pool2_1_ = self.pool2_1_(lrn2_1_)
        relu2_1_ = self.relu2_1_(pool2_1_)
        get2_2_ = split1_[1]
        conv2_2_padding = self.conv2_2_padding(get2_2_)
        conv2_2_ = self.conv2_2_(conv2_2_padding)
        lrn2_2_ = F.LRN(data=conv2_2_,
            alpha=0.0001,
            beta=0.75,
            knorm=2,
            nsize=5)
        pool2_2_ = self.pool2_2_(lrn2_2_)
        relu2_2_ = self.relu2_2_(pool2_2_)
        concatenate3_ = self.concatenate3_(relu2_1_, relu2_2_)
        conv3_padding = self.conv3_padding(concatenate3_)
        conv3_ = self.conv3_(conv3_padding)
        relu3_ = self.relu3_(conv3_)
        split3_ = self.split3_(relu3_)
        get4_1_ = split3_[0]
        conv4_1_padding = self.conv4_1_padding(get4_1_)
        conv4_1_ = self.conv4_1_(conv4_1_padding)
        relu4_1_ = self.relu4_1_(conv4_1_)
        conv5_1_padding = self.conv5_1_padding(relu4_1_)
        conv5_1_ = self.conv5_1_(conv5_1_padding)
        pool5_1_ = self.pool5_1_(conv5_1_)
        relu5_1_ = self.relu5_1_(pool5_1_)
        get4_2_ = split3_[1]
        conv4_2_padding = self.conv4_2_padding(get4_2_)
        conv4_2_ = self.conv4_2_(conv4_2_padding)
        relu4_2_ = self.relu4_2_(conv4_2_)
        conv5_2_padding = self.conv5_2_padding(relu4_2_)
        conv5_2_ = self.conv5_2_(conv5_2_padding)
        pool5_2_ = self.pool5_2_(conv5_2_)
        relu5_2_ = self.relu5_2_(pool5_2_)
        concatenate6_ = self.concatenate6_(relu5_1_, relu5_2_)
        fc6_flatten_ = self.fc6_flatten(concatenate6_)
        fc6_ = self.fc6_(fc6_flatten_)
        relu6_ = self.relu6_(fc6_)
        dropout6_ = self.dropout6_(relu6_)
        fc7_ = self.fc7_(dropout6_)
        relu7_ = self.relu7_(fc7_)
        dropout7_ = self.dropout7_(relu7_)
        fc8_ = self.fc8_(dropout7_)
        softmax8_ = self.softmax8_(fc8_)
        outputs.append(softmax8_)

        return outputs[0]
