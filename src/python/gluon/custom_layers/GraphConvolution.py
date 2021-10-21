import mxnet as mx
from mxnet import gluon
from mxnet.gluon import nn


class GraphConvolution(gluon.HybridBlock):
    def __init__(self, input_dim, output_dim, adjacency, **kwargs):
        super(GraphConvolution, self).__init__(**kwargs)
        with self.name_scope():
            self.weight = self.params.get('weight', init=mx.init.Uniform(), shape=(input_dim, output_dim))
            self.adjacency = adjacency
            self.input_dim = input_dim
            self.output_dim = output_dim

    def hybrid_forward(self, F, x, *args, **kwargs):
        x = mx.np.matmul(x, self.weight.data())
        x = mx.np.matmul(self.adjacency, x)
        return x

    def get_parameters(self):
        # return {'weight': self.params.get('weight').dtype}
        return {'input_dim': int, 'output_dim': int, 'adjacency': int}

    def print_parameters(self):
        print({'input_dim': int, 'output_dim': int, 'adjacency': int})

    def compute_output_types(self, input_dim, output_dim, adjacency, input_dimension_array):
        channels1 = input_dimension_array[0][0]
        height1 = input_dimension_array[0][1]
        width1 = input_dimension_array[0][2]
        min1 = 0
        max1 = 'oo'
        print([(channels1, height1, width1, min1, max1)])

