import sys
sys.path.insert(1, './src')

import getopt

import numpy as np
import mxnet as mx
from mxnet import nd, autograd
from mxnet import gluon
from mxnet.gluon import nn
from mxnet.test_utils import get_mnist_iterator
import logging

from models.small_seg import SmallSeg
from models.fcn_vgg16 import FcnVGG16



def make_graph(filename, title):
    model = FcnVGG16()
    model.hybridize()

    symbol_data = mx.sym.var('data')

    tmp = model(symbol_data)

    mx.viz.print_summary(tmp)

    digraph = mx.viz.plot_network(tmp, title=title)
    digraph.view(filename=filename)


def main(argv):
    """ Main entry point to the program. """

    modelname = 'FcnVGG16'

    output_file = 'documentation/' + modelname
    title = modelname

    try:
        opts, args = getopt.getopt(argv, 'o:t:', ["ofile=", "title="])
    except getopt.GetoptError:
        print("make_graph.py -o <output_file> -t <title>")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("crfasrnn_demo.py -i <inputfile> -o <outputfile> -g <gpu_device>")
            sys.exit()
        elif opt in ("-o", "ofile"):
            output_file = arg
        elif opt in ("-t", "title"):
            title = arg

    print("Output file: {}".format(output_file))
    print("Title: {}".format(title))
    print("Model: {}".format(modelname))
    make_graph(output_file, title)


if __name__ == "__main__":
    main(sys.argv[1:])