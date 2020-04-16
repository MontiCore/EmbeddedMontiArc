import sys
sys.path.insert(1, './src')

import os
import getopt

import numpy as np
import mxnet as mx
from mxnet import nd, autograd
from mxnet import gluon
from mxnet.gluon import nn
from mxnet.test_utils import get_mnist_iterator
import logging


def make_graph(model, filename, title='network'):
    symbol_data = mx.sym.var('data')

    tmp = model(symbol_data)

    mx.viz.print_summary(tmp)

    digraph = mx.viz.plot_network(tmp, title=title)
    digraph.view(filename=filename)


def test_input(model, shape=(32,3,480,480), ctx=mx.gpu(0)):
    x = mx.nd.random.uniform(shape=shape, ctx=ctx)
    print('Input shape: ', x.shape)
    outputs = model(x)
    print('Output shape: ', outputs.shape)

def main(argv):
    filename= 'graph'
    model_path = './model/cNNSegment.MediumSeg/model_0_newest-symbol.json'
    params_path = './model/cNNSegment.MediumSeg/model_0_newest-0000.params'
    director_path = '.model/cNNSegment.SmallSeg/'
    ctx = mx.gpu(0)
    size = 28

    ### parse_args
    try:
        opts, _ = getopt.getopt(argv, 'f:c:m:p:d:', ["filename=","ctx=","model=","params=","dir="])
    except getopt.GetoptError:
        print("demo.py -f <filename> -c <context> -m <model> -p <params> -d <dir>")
        sys.exit(2)


    for opt, arg in opts:
        if opt == '-h':
            print("demo.py -f <filename> -c <context> -m <model> -p <params> -d <dir>")
            sys.exit()
        elif opt in ("-f", "filename"):
            filename = arg
        elif opt in ("-c", "ctx"):
            if arg == 'gpu':
                ctx = mx.gpu(0)
            elif arg == 'cpu':
                ctx = mx.cpu(0)
        elif opt in ("-d", "dir"):
            model_path = os.path.join(arg, 'model_0_newest-symbol.json')
            params_path = os.path.join(arg, 'model_0_newest-0000.params')
        elif opt in ("-p", "params"):
            params_path = arg
        elif opt in ("-m", "model"):
            model_path = arg

    """
    https://discuss.mxnet.io/t/collect-params-load-model-params-file-does-not-work/1754

    https://discuss.mxnet.io/t/save-and-load-params-gluon/4050/3
    """

    ### load model and parameters
    model = gluon.nn.SymbolBlock.imports(model_path, ['data'], params_path, ctx=ctx)

    make_graph(model, filename=filename)


    ### el final
if __name__ == "__main__":
    main(sys.argv[1:])