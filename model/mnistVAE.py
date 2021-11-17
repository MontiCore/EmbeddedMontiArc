import os
import shutil
import sys
import warnings

import mxnet as mx
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, autograd, gluon
from mxnet.gluon import nn
import cv2

if __name__ == "__main__":
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        net = gluon.nn.SymbolBlock.imports("model_0_newest-symbol.json", ['data'],
                                                        "model_0_newest-0000.params", ctx=mx.cpu())

    sample = mx.ndarray.random_normal(0,1,(1,2))
    res = mx.ndarray.transpose(net(sample).squeeze(0)).asnumpy()
    print(res.shape)
    cv2.imshow("Generated", res)
    cv2.waitKey(0)