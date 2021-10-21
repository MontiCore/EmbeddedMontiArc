from __future__ import print_function
from tkinter.filedialog import test

import mxnet as mx
import mxnet.ndarray as nd
from mxnet import nd, autograd, gluon
from mxnet.gluon.data.vision import transforms
import h5py
from mxnet.visualization import plot_network

import numpy as np
import warnings

SMALL = False
NAMES = ["SenetSmallFC"]

for name in NAMES:
    net = gluon.nn.SymbolBlock.imports("../../model/" + name + \
        "/model_0_newest-symbol.json", ['data'], "../../model/" + name + \
        "/model_0-0000.params")
    if NAMES:
        x = mx.nd.zeros((1, 3, 64 ,64))
    else:
         x = mx.nd.zeros((1, 3, 210 ,280))
    #print(net(x))
    net.summary(x)
#net.hybrid_forward(x)
#digraph = mx.viz.plot_network(net, shape={'data':(1, 3, 64, 64)}, node_attrs={"fixedsize":"false"})
#digraph.view()


#mx.viz.plot_network(net(mx.sym.var('data'))[0], node_attrs={"shape":"oval","fixedsize":"false"})