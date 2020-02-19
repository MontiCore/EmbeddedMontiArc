import sys
import os
import getopt
import cv2
import mxnet as mx
from mxnet import gluon
import numpy as np

from util import get_preprocessed_image_mnist, get_label_image, _PALETTE

def plot(img, labels, class_names=None, title='pred.png'):
    import matplotlib.pyplot as plt
    from matplotlib import patches

    if class_names is None:
        ax = plt.subplot(121)
        ax.imshow(img)
        ax2 = plt.subplot(122)
        ax2.imshow(labels)
    if class_names is not None:
        ax = plt.subplot(131)
        ax.imshow(img)
        ax2 = plt.subplot(132)
        ax2.imshow(labels)
        _palette = [x/255.0 for x in _PALETTE]
        palette = [tuple(_palette[n:n+3]) for n in range(0, len(_palette), 3)]
        handles = [patches.Patch(color=color, label=label) for color, label in zip(palette, class_names)]
        ax3 = plt.subplot(133)
        ax3.axis('off')
        plt.legend(handles=handles)

    plt.savefig(title)
    plt.show()

def main(argv):
    model_path = './model/cNNSegment.SmallSeg/model_0_newest-symbol.json'
    params_path = './model/cNNSegment.SmallSeg/model_0_newest-0000.params'
    input_file = '../resources/images_fashionmnist/3.png'
    output_file = 'pred.png'
    ctx = mx.gpu(0)

    ### parse_args
    try:
        opts, _ = getopt.getopt(argv, 'i:o:c:p:', ["ifile=","ofile=","ctx=","params="])
    except getopt.GetoptError:
        print("demo.py -i <input_file> -o <output_file> -c <context> -p <params>")
        sys.exit(2)


    for opt, arg in opts:
        if opt == '-h':
            print("demo.py -i <input_file> -o <output_file> -c <context> -p <params>")
            sys.exit()
        elif opt in ("-i", "ifile"):
            input_file = arg
        elif opt in ("-o", "ofile"):
            output_file = arg
        elif opt in ("-c", "ctx"):
            if arg == 'gpu':
                ctx = mx.gpu(0)
            elif arg == 'cpu':
                ctx = mx.cpu(0)
        elif opt in ("-o", "ofile"):
            params_path = arg

    """
    https://discuss.mxnet.io/t/collect-params-load-model-params-file-does-not-work/1754

    https://discuss.mxnet.io/t/save-and-load-params-gluon/4050/3
    """

    ### load model
    # net = SmallSeg()

    ### load weights
    # net.load_parameters(params_path, ctx=ctx)

    ### load model and parameters
    net = gluon.nn.SymbolBlock.imports(model_path, ['data'], params_path, ctx=ctx)

    ### load preprocess image
    img_processed, org_h, org_w = get_preprocessed_image_mnist(input_file)

    print(img_processed.shape)

    # ### get labels
    output = net(mx.nd.array(img_processed,ctx=ctx))

    # print(output)

    # ### get label image (apply a color palette for labels)
    labels = get_label_image(output, org_h, org_w)

    # ### save image
    cv2.imwrite(output_file, labels)
    class_names = ['background', 'trousers', 'pullover', 'dress', 'coat', 'sandal', 'shirt', 'sneaker', 'bag', 'ankle_boot', 't-shirt']
    plot(img_processed[0][0], labels, class_names=class_names, title=output_file)


    ### el final

if __name__ == '__main__':
    main(sys.argv[1:])