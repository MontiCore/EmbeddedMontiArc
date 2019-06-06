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
        self.last_layers = {}
        with self.name_scope():
            if data_mean:
                assert(data_std)
                self.input_normalization_data = ZScoreNormalization(data_mean=data_mean['data'],
                                                                               data_std=data_std['data'])
            else:
                self.input_normalization_data = NoNormalization()

            self.conv2_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv2_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv2_1_, output shape: {[8,32,32]}

            self.batchnorm2_1_ = gluon.nn.BatchNorm()
            # batchnorm2_1_, output shape: {[8,32,32]}

            self.relu2_1_ = gluon.nn.Activation(activation='relu')
            self.conv3_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv3_1_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv3_1_, output shape: {[8,32,32]}

            self.batchnorm3_1_ = gluon.nn.BatchNorm()
            # batchnorm3_1_, output shape: {[8,32,32]}

            self.conv2_2_ = gluon.nn.Conv2D(channels=8,
                kernel_size=(1,1),
                strides=(1,1),
                use_bias=True)
            # conv2_2_, output shape: {[8,32,32]}

            self.batchnorm2_2_ = gluon.nn.BatchNorm()
            # batchnorm2_2_, output shape: {[8,32,32]}

            self.relu4_ = gluon.nn.Activation(activation='relu')
            self.conv5_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv5_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv5_1_, output shape: {[16,16,16]}

            self.batchnorm5_1_ = gluon.nn.BatchNorm()
            # batchnorm5_1_, output shape: {[16,16,16]}

            self.relu5_1_ = gluon.nn.Activation(activation='relu')
            self.conv6_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv6_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv6_1_, output shape: {[16,16,16]}

            self.batchnorm6_1_ = gluon.nn.BatchNorm()
            # batchnorm6_1_, output shape: {[16,16,16]}

            self.conv5_2_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv5_2_, output shape: {[16,16,16]}

            self.batchnorm5_2_ = gluon.nn.BatchNorm()
            # batchnorm5_2_, output shape: {[16,16,16]}

            self.relu7_ = gluon.nn.Activation(activation='relu')
            self.conv8_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv8_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv8_1_, output shape: {[16,16,16]}

            self.batchnorm8_1_ = gluon.nn.BatchNorm()
            # batchnorm8_1_, output shape: {[16,16,16]}

            self.relu8_1_ = gluon.nn.Activation(activation='relu')
            self.conv9_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv9_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv9_1_, output shape: {[16,16,16]}

            self.batchnorm9_1_ = gluon.nn.BatchNorm()
            # batchnorm9_1_, output shape: {[16,16,16]}

            self.relu10_ = gluon.nn.Activation(activation='relu')
            self.conv11_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv11_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv11_1_, output shape: {[16,16,16]}

            self.batchnorm11_1_ = gluon.nn.BatchNorm()
            # batchnorm11_1_, output shape: {[16,16,16]}

            self.relu11_1_ = gluon.nn.Activation(activation='relu')
            self.conv12_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv12_1_ = gluon.nn.Conv2D(channels=16,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv12_1_, output shape: {[16,16,16]}

            self.batchnorm12_1_ = gluon.nn.BatchNorm()
            # batchnorm12_1_, output shape: {[16,16,16]}

            self.relu13_ = gluon.nn.Activation(activation='relu')
            self.conv14_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv14_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv14_1_, output shape: {[32,8,8]}

            self.batchnorm14_1_ = gluon.nn.BatchNorm()
            # batchnorm14_1_, output shape: {[32,8,8]}

            self.relu14_1_ = gluon.nn.Activation(activation='relu')
            self.conv15_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv15_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv15_1_, output shape: {[32,8,8]}

            self.batchnorm15_1_ = gluon.nn.BatchNorm()
            # batchnorm15_1_, output shape: {[32,8,8]}

            self.conv14_2_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv14_2_, output shape: {[32,8,8]}

            self.batchnorm14_2_ = gluon.nn.BatchNorm()
            # batchnorm14_2_, output shape: {[32,8,8]}

            self.relu16_ = gluon.nn.Activation(activation='relu')
            self.conv17_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv17_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv17_1_, output shape: {[32,8,8]}

            self.batchnorm17_1_ = gluon.nn.BatchNorm()
            # batchnorm17_1_, output shape: {[32,8,8]}

            self.relu17_1_ = gluon.nn.Activation(activation='relu')
            self.conv18_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv18_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv18_1_, output shape: {[32,8,8]}

            self.batchnorm18_1_ = gluon.nn.BatchNorm()
            # batchnorm18_1_, output shape: {[32,8,8]}

            self.relu19_ = gluon.nn.Activation(activation='relu')
            self.conv20_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv20_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv20_1_, output shape: {[32,8,8]}

            self.batchnorm20_1_ = gluon.nn.BatchNorm()
            # batchnorm20_1_, output shape: {[32,8,8]}

            self.relu20_1_ = gluon.nn.Activation(activation='relu')
            self.conv21_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv21_1_ = gluon.nn.Conv2D(channels=32,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv21_1_, output shape: {[32,8,8]}

            self.batchnorm21_1_ = gluon.nn.BatchNorm()
            # batchnorm21_1_, output shape: {[32,8,8]}

            self.relu22_ = gluon.nn.Activation(activation='relu')
            self.conv23_1_padding = Padding(padding=(0,0,0,0,1,0,1,0))
            self.conv23_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(2,2),
                use_bias=True)
            # conv23_1_, output shape: {[64,4,4]}

            self.batchnorm23_1_ = gluon.nn.BatchNorm()
            # batchnorm23_1_, output shape: {[64,4,4]}

            self.relu23_1_ = gluon.nn.Activation(activation='relu')
            self.conv24_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv24_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv24_1_, output shape: {[64,4,4]}

            self.batchnorm24_1_ = gluon.nn.BatchNorm()
            # batchnorm24_1_, output shape: {[64,4,4]}

            self.conv23_2_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(1,1),
                strides=(2,2),
                use_bias=True)
            # conv23_2_, output shape: {[64,4,4]}

            self.batchnorm23_2_ = gluon.nn.BatchNorm()
            # batchnorm23_2_, output shape: {[64,4,4]}

            self.relu25_ = gluon.nn.Activation(activation='relu')
            self.conv26_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv26_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv26_1_, output shape: {[64,4,4]}

            self.batchnorm26_1_ = gluon.nn.BatchNorm()
            # batchnorm26_1_, output shape: {[64,4,4]}

            self.relu26_1_ = gluon.nn.Activation(activation='relu')
            self.conv27_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv27_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv27_1_, output shape: {[64,4,4]}

            self.batchnorm27_1_ = gluon.nn.BatchNorm()
            # batchnorm27_1_, output shape: {[64,4,4]}

            self.relu28_ = gluon.nn.Activation(activation='relu')
            self.conv29_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv29_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv29_1_, output shape: {[64,4,4]}

            self.batchnorm29_1_ = gluon.nn.BatchNorm()
            # batchnorm29_1_, output shape: {[64,4,4]}

            self.relu29_1_ = gluon.nn.Activation(activation='relu')
            self.conv30_1_padding = Padding(padding=(0,0,0,0,1,1,1,1))
            self.conv30_1_ = gluon.nn.Conv2D(channels=64,
                kernel_size=(3,3),
                strides=(1,1),
                use_bias=True)
            # conv30_1_, output shape: {[64,4,4]}

            self.batchnorm30_1_ = gluon.nn.BatchNorm()
            # batchnorm30_1_, output shape: {[64,4,4]}

            self.relu31_ = gluon.nn.Activation(activation='relu')
            self.globalpooling31_ = gluon.nn.GlobalAvgPool2D()
            # globalpooling31_, output shape: {[64,1,1]}

            self.fc31_ = gluon.nn.Dense(units=128, use_bias=True)
            # fc31_, output shape: {[128,1,1]}

            self.dropout31_ = gluon.nn.Dropout(rate=0.5)
            self.fc32_ = gluon.nn.Dense(units=10, use_bias=True)
            # fc32_, output shape: {[10,1,1]}

        self.last_layers['softmax'] = 'softmax'


    def hybrid_forward(self, F, data):
        data = self.input_normalization_data(data)
        conv2_1_padding = self.conv2_1_padding(data)
        conv2_1_ = self.conv2_1_(conv2_1_padding)
        batchnorm2_1_ = self.batchnorm2_1_(conv2_1_)
        relu2_1_ = self.relu2_1_(batchnorm2_1_)
        conv3_1_padding = self.conv3_1_padding(relu2_1_)
        conv3_1_ = self.conv3_1_(conv3_1_padding)
        batchnorm3_1_ = self.batchnorm3_1_(conv3_1_)
        conv2_2_ = self.conv2_2_(data)
        batchnorm2_2_ = self.batchnorm2_2_(conv2_2_)
        add4_ = batchnorm3_1_ + batchnorm2_2_
        relu4_ = self.relu4_(add4_)
        conv5_1_padding = self.conv5_1_padding(relu4_)
        conv5_1_ = self.conv5_1_(conv5_1_padding)
        batchnorm5_1_ = self.batchnorm5_1_(conv5_1_)
        relu5_1_ = self.relu5_1_(batchnorm5_1_)
        conv6_1_padding = self.conv6_1_padding(relu5_1_)
        conv6_1_ = self.conv6_1_(conv6_1_padding)
        batchnorm6_1_ = self.batchnorm6_1_(conv6_1_)
        conv5_2_ = self.conv5_2_(relu4_)
        batchnorm5_2_ = self.batchnorm5_2_(conv5_2_)
        add7_ = batchnorm6_1_ + batchnorm5_2_
        relu7_ = self.relu7_(add7_)
        conv8_1_padding = self.conv8_1_padding(relu7_)
        conv8_1_ = self.conv8_1_(conv8_1_padding)
        batchnorm8_1_ = self.batchnorm8_1_(conv8_1_)
        relu8_1_ = self.relu8_1_(batchnorm8_1_)
        conv9_1_padding = self.conv9_1_padding(relu8_1_)
        conv9_1_ = self.conv9_1_(conv9_1_padding)
        batchnorm9_1_ = self.batchnorm9_1_(conv9_1_)
        add10_ = batchnorm9_1_ + relu7_
        relu10_ = self.relu10_(add10_)
        conv11_1_padding = self.conv11_1_padding(relu10_)
        conv11_1_ = self.conv11_1_(conv11_1_padding)
        batchnorm11_1_ = self.batchnorm11_1_(conv11_1_)
        relu11_1_ = self.relu11_1_(batchnorm11_1_)
        conv12_1_padding = self.conv12_1_padding(relu11_1_)
        conv12_1_ = self.conv12_1_(conv12_1_padding)
        batchnorm12_1_ = self.batchnorm12_1_(conv12_1_)
        add13_ = batchnorm12_1_ + relu10_
        relu13_ = self.relu13_(add13_)
        conv14_1_padding = self.conv14_1_padding(relu13_)
        conv14_1_ = self.conv14_1_(conv14_1_padding)
        batchnorm14_1_ = self.batchnorm14_1_(conv14_1_)
        relu14_1_ = self.relu14_1_(batchnorm14_1_)
        conv15_1_padding = self.conv15_1_padding(relu14_1_)
        conv15_1_ = self.conv15_1_(conv15_1_padding)
        batchnorm15_1_ = self.batchnorm15_1_(conv15_1_)
        conv14_2_ = self.conv14_2_(relu13_)
        batchnorm14_2_ = self.batchnorm14_2_(conv14_2_)
        add16_ = batchnorm15_1_ + batchnorm14_2_
        relu16_ = self.relu16_(add16_)
        conv17_1_padding = self.conv17_1_padding(relu16_)
        conv17_1_ = self.conv17_1_(conv17_1_padding)
        batchnorm17_1_ = self.batchnorm17_1_(conv17_1_)
        relu17_1_ = self.relu17_1_(batchnorm17_1_)
        conv18_1_padding = self.conv18_1_padding(relu17_1_)
        conv18_1_ = self.conv18_1_(conv18_1_padding)
        batchnorm18_1_ = self.batchnorm18_1_(conv18_1_)
        add19_ = batchnorm18_1_ + relu16_
        relu19_ = self.relu19_(add19_)
        conv20_1_padding = self.conv20_1_padding(relu19_)
        conv20_1_ = self.conv20_1_(conv20_1_padding)
        batchnorm20_1_ = self.batchnorm20_1_(conv20_1_)
        relu20_1_ = self.relu20_1_(batchnorm20_1_)
        conv21_1_padding = self.conv21_1_padding(relu20_1_)
        conv21_1_ = self.conv21_1_(conv21_1_padding)
        batchnorm21_1_ = self.batchnorm21_1_(conv21_1_)
        add22_ = batchnorm21_1_ + relu19_
        relu22_ = self.relu22_(add22_)
        conv23_1_padding = self.conv23_1_padding(relu22_)
        conv23_1_ = self.conv23_1_(conv23_1_padding)
        batchnorm23_1_ = self.batchnorm23_1_(conv23_1_)
        relu23_1_ = self.relu23_1_(batchnorm23_1_)
        conv24_1_padding = self.conv24_1_padding(relu23_1_)
        conv24_1_ = self.conv24_1_(conv24_1_padding)
        batchnorm24_1_ = self.batchnorm24_1_(conv24_1_)
        conv23_2_ = self.conv23_2_(relu22_)
        batchnorm23_2_ = self.batchnorm23_2_(conv23_2_)
        add25_ = batchnorm24_1_ + batchnorm23_2_
        relu25_ = self.relu25_(add25_)
        conv26_1_padding = self.conv26_1_padding(relu25_)
        conv26_1_ = self.conv26_1_(conv26_1_padding)
        batchnorm26_1_ = self.batchnorm26_1_(conv26_1_)
        relu26_1_ = self.relu26_1_(batchnorm26_1_)
        conv27_1_padding = self.conv27_1_padding(relu26_1_)
        conv27_1_ = self.conv27_1_(conv27_1_padding)
        batchnorm27_1_ = self.batchnorm27_1_(conv27_1_)
        add28_ = batchnorm27_1_ + relu25_
        relu28_ = self.relu28_(add28_)
        conv29_1_padding = self.conv29_1_padding(relu28_)
        conv29_1_ = self.conv29_1_(conv29_1_padding)
        batchnorm29_1_ = self.batchnorm29_1_(conv29_1_)
        relu29_1_ = self.relu29_1_(batchnorm29_1_)
        conv30_1_padding = self.conv30_1_padding(relu29_1_)
        conv30_1_ = self.conv30_1_(conv30_1_padding)
        batchnorm30_1_ = self.batchnorm30_1_(conv30_1_)
        add31_ = batchnorm30_1_ + relu28_
        relu31_ = self.relu31_(add31_)
        globalpooling31_ = self.globalpooling31_(relu31_)
        fc31_ = self.fc31_(globalpooling31_)
        dropout31_ = self.dropout31_(fc31_)
        fc32_ = self.fc32_(dropout31_)
        return fc32_

