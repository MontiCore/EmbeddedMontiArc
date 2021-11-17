import time
import numpy as np
import mxnet as mx
from mxnet import nd, autograd, gluon
from mxnet.gluon import nn
import matplotlib.pyplot as plt

class Test(gluon.nn.HybridBlock):
    def __init__(self):
        super().__init__()
        self.embedding = gluon.nn.Embedding(64,16,weight_initializer=mx.init.Uniform())

    def hybrid_forward(self, F, x, *args, **kwargs):
        e = self.embedding(x)
        return e

if __name__ == "__main__":
    net = Test()
    net.collect_params().initialize(mx.init.Uniform(), force_reinit=False, ctx=mx.cpu())
    net.hybridize()
    net_trainer = mx.gluon.Trainer(net.collect_params(), optimizer='adam', optimizer_params={'learning_rate': 0.001})
    e = net(mx.ndarray.zeros((9800,16)))
    nd.waitall()
    net.export("das geht wahrscheinlich",epoch=0)





