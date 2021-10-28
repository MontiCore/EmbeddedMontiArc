import mxnet as mx
from mxnet import gluon, nd, util
from mxnet.gluon import HybridBlock, nn


class GraphConvolution(HybridBlock):
    def __init__(self, input_dim=1433, output_dim=7, **kwargs):
        super(GraphConvolution, self).__init__(**kwargs)
        self.weight = self.params.get('weight', init=mx.init.Uniform(), shape=(input_dim, output_dim))
        self.input_dim = input_dim
        self.output_dim = output_dim

    def hybrid_forward(self, F, adjacency, features, weight, *args, **kwargs):
        x = F.batch_dot(adjacency, features)
        x = F.dot(x, weight)
        return x

    def get_parameters(self):
        return {'input_dim': int, 'output_dim': int, 'weight': self.weight.dtype}

    def print_parameters(self):
        print({'input_dim': int, 'output_dim': int, 'weight': self.weight.dtype})

    def compute_output_types(self, input_dim, output_dim, input_dimension_array):
        channels1 = 1
        height1 = 2708
        width1 = output_dim
        min1 = 0
        max1 = 'oo'
        print([(channels1, height1, width1, min1, max1)])
