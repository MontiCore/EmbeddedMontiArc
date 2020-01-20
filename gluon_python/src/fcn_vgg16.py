import numpy as np
import mxnet as mx
from mxnet import nd, autograd, init
from mxnet import gluon
from mxnet.gluon import nn
from mxnet.test_utils import get_mnist_iterator
import logging

from custom_layers import CroppingLayer2D, MaxPool2DSamePadding


class FcnVGG16(nn.HybridBlock):
    def __init__(self, input_shape=(3,480,480), num_classes=21, **kwargs):
        super(FcnVGG16, self).__init__(**kwargs)
        # Input
        self.input_shape = input_shape[::-1]
        self.num_classes = num_classes

        # pad_0 = 50
        pad_0 = 120
        pad_skip4_1 = 6
        pad_skip4_2 = -7
        pad_skip3_1 = 13
        pad_skip3_2 = -13
        pad_end_1 = 12
        pad_end_2 = -12

        # VGG-16 convolution block 1
        self.block1 = nn.HybridSequential()
        with self.block1.name_scope():
            '''Achieve 'same' padding same as in tensorflow keras:
            https://discuss.mxnet.io/t/pooling-and-convolution-with-same-mode/528/2
            Essentially: kernel_size=(k, k) -> padding=(k//2, k//2)
            If k is even, slice off first column and row.
            if k%2 == 0:
                pool = pool[:,:,1:,1:]
            '''

            self.block1.add(nn.Conv2D(64, (3, 3), activation='relu', padding=(pad_0,pad_0)))
            self.block1.add(nn.Conv2D(64, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block1.add(MaxPool2DSamePadding((2, 2), strides=(2, 2)))

        # VGG-16 convolution block 2
        self.block2 = nn.HybridSequential()
        with self.block2.name_scope():
            self.block2.add(self.block1)
            self.block2.add(nn.Conv2D(128, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block2.add(nn.Conv2D(128, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block2.add(MaxPool2DSamePadding((2, 2), strides=(2, 2)))

        # VGG-16 convolution block 3
        self.block3 = nn.HybridSequential()
        with self.block3.name_scope():
            self.block3.add(self.block2)
            self.block3.add(nn.Conv2D(256, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block3.add(nn.Conv2D(256, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block3.add(nn.Conv2D(256, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block3.add(MaxPool2DSamePadding((2, 2), strides=(2, 2)))

        # VGG-16 convolution block 4
        self.block4 = nn.HybridSequential()
        with self.block4.name_scope():
            self.block4.add(nn.Conv2D(512, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block4.add(nn.Conv2D(512, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block4.add(nn.Conv2D(512, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block4.add(MaxPool2DSamePadding((2, 2), strides=(2, 2)))

        # VGG-16 convolution block 5
        self.block5 = nn.HybridSequential()
        with self.block5.name_scope():
            self.block5.add(nn.Conv2D(512, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block5.add(nn.Conv2D(512, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block5.add(nn.Conv2D(512, (3, 3), activation='relu', padding=(3//2, 3//2)))
            self.block5.add(MaxPool2DSamePadding((2, 2), strides=(2, 2)))

        # Fully-connected layers converted to convolution layers
        self.fc_end = nn.HybridSequential()
        with self.fc_end.name_scope():
            self.fc_end.add(self.block5)
            self.fc_end.add(nn.Conv2D(4096, (7, 7), activation='relu'))
            self.fc_end.add(nn.Dropout(0.5))
            self.fc_end.add(nn.Conv2D(4096, (1, 1), activation='relu'))
            self.fc_end.add(nn.Dropout(0.5))
            self.fc_end.add(nn.Conv2D(self.num_classes, (1, 1)))

        # Deconvolution
        self.deconv = nn.HybridSequential()
        with self.deconv.name_scope():
            self.deconv.add(self.fc_end)
            self.deconv.add(nn.Conv2DTranspose(self.num_classes, (4, 4), strides=2))

        # Skip connections from pool4
        self.skip_4 = nn.HybridSequential()
        with self.skip_4.name_scope():
            self.skip_4.add(nn.Conv2D(self.num_classes, (1, 1)))
            self.skip_4.add(CroppingLayer2D((None, None, pad_skip4_1, pad_skip4_1),(None, None, pad_skip4_2, pad_skip4_2)))

        self.conc1 = nn.HybridSequential()
        with self.conc1.name_scope():
            self.conc1.add(nn.Conv2DTranspose(self.num_classes, (4, 4), strides=2, use_bias=False))

        # Skip connections from pool3
        self.skip_3 = nn.HybridSequential()
        with self.skip_3.name_scope():
            self.skip_3.add(nn.Conv2D(self.num_classes, (1, 1)))
            self.skip_3.add(CroppingLayer2D((None, None, pad_skip3_1, pad_skip3_1),(None, None, pad_skip3_2, pad_skip3_2)))

        # Final up-sampling and cropping
        self.out = nn.HybridSequential()
        with self.out.name_scope():
            self.out.add(nn.Conv2DTranspose(self.num_classes, (16, 16), strides=8, use_bias=False))
            self.out.add(CroppingLayer2D((None, None, pad_end_1, pad_end_1), (None, None, pad_end_2, pad_end_2)))

    def hybrid_forward(self, F, X):
        # print('Type of X', type(X))
        # arg_shape, output_shape, aux_shape = self.block1.infer_shape()
        # print(arg_shape(output_shape, aux_shape))

        out_block3 = self.block3(X)
        out_block4 = self.block4(out_block3)
        out_block5_fc_deconv = self.deconv(out_block4)
        out_skip4 = self.skip_4(out_block4)
        out_skip3 = self.skip_3(out_block3)
        out_conc1 = self.conc1(out_block5_fc_deconv + out_skip4)
        out = self.out(out_skip3 + out_conc1)
        # out = F.softmax(out)

        # print(out_block5_fc_deconv.list_outputs())
        return out

if __name__ == '__main__':
    model = FcnVGG16()
    model.hybridize()
    model.initialize(init=init.Xavier())

    ### dummy data
    x = mx.nd.random.uniform(shape=(1, 3, 480, 480))
    outputs = model(x)
