import logging
import os
import random
import sys
sys.path.insert(1, './src')

from datetime import datetime

import gluoncv.data
import h5py
import mxnet as mx
import numpy as np
from gluoncv.loss import MixSoftmaxCrossEntropyLoss
from gluoncv.utils.parallel import DataParallelCriterion, DataParallelModel
from gluoncv.utils.viz import DeNormalize, get_color_pallete
from matplotlib import pyplot as plt
from mxnet import autograd, gluon, init, nd
from mxnet.gluon import nn
from mxnet.gluon.data.vision import transforms
import cv2

from train.util import visualize_data


_ctx = mx.gpu(0)

# logging.basicConfig(,,level=logging.DEBUG)
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

def test_input(model, shape=(1,3,480,480)):
    x = mx.nd.random.uniform(shape=shape, ctx=_ctx)
    print('Input shape: ', x.shape)
    outputs = model(x)
    print('Output shape: ', outputs.shape)

def acc(output, label):
    # output: (batch, num_output) float32 ndarray
    # label: (batch, ) int32 ndarray
    return (output.argmax(axis=1) ==
            label.astype('float32')).mean().asscalar()

def load_data(data='VOC', split='train', batch_size=32):
    input_transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
    ])
    if data == 'VOC':
        trainset = gluoncv.data.VOCSegmentation(split=split, transform=input_transform)
    # trainset = mx.nd.array(trainset, ctx=ctx)
    print('Training images:', len(trainset))
    # Create Training Loader
    train_data = gluon.data.DataLoader(
        trainset, batch_size, shuffle=True, last_batch='rollover',
        num_workers=batch_size)

    return trainset, train_data

def test_stuff():
    trainset, train_data = load_data(data='VOC', split='train', batch_size=2)

    if True:
        random.seed(datetime.now())
        idx = 1 # random.randint(0, len(trainset))
        image, mask = trainset[idx]
        # visualize_data(*trainset[idx])
        visualize_data(image, mask)

    for i, (data, target) in enumerate(train_data):
        print('Shape input: {}'.format(data.shape))
        print('Shape target: {}'.format(target.shape))
        if i > 5:
            break

if __name__ == '__main__':
    test_stuff()
