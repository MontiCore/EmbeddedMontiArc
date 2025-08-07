from __future__ import print_function
from tkinter.filedialog import test

import mxnet as mx
import mxnet.ndarray as nd
from mxnet import nd, autograd, gluon
from mxnet.gluon.data.vision import transforms
import h5py
from PIL import Image

import numpy as np
import warnings

SMALL = False
NAMES = ["cv.nn.AlexnetSmall"]

for name in NAMES:
    net = gluon.nn.SymbolBlock.imports(
        symbol_file="../../model/" + name + "/model_0_newest-symbol.json", \
        input_names=['data'], \
        param_file="../../model/" + name + "/model_0_newest-0000.params")
    if NAMES:
        x = mx.nd.zeros((1, 3, 64 ,64))
    else:
         x = mx.nd.zeros((1, 3, 210 ,280))

    net.summary(x)

debuggin_dataset_path = "../../training-data-debugging-small"
debugging_train_file = h5py.File(debuggin_dataset_path + "/train.h5", "r")

train_entry = np.random.randint(750)
img = Image.fromarray(np.moveaxis(debugging_train_file["data"][train_entry], [0], [2]), 'RGB')
img.show()

input = mx.nd.array([debugging_train_file["data"][train_entry]])
label = debugging_train_file["predictions_label"][train_entry]

pred = net.forward(input)[0].asnumpy()
mae = sum(abs(pred - label)) / 14
l2 = np.sqrt(sum((pred - label) ** 2))

print(mae)
print(l2)

 