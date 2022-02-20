import mxnet as mx
import numpy as np
import warnings
import h5py
import matplotlib.pyplot as plt
from os.path import expanduser
from mxnet import gluon

def get_data(batch_size):
    mx_context = mx.cpu()
    home = expanduser("~")
    data_dir = home + "/.m2/repository/de/monticore/lang/monticar/datasets/vae-mnist/1/vae-mnist-1-dataset/training_data/"
    train_path = data_dir + "train.h5"
    test_path = data_dir + "test.h5"

    train_h5 = h5py.File(train_path, 'r')
    test_h5 = h5py.File(test_path, 'r')

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        encoder = gluon.nn.SymbolBlock.imports(home+"/wrk/vae/mnist-vae/vq-vae/pre-trained/model/vqvae.Encoder/model_0_newest-symbol.json",
                                               ['data'],
                                               home+"/wrk/vae/mnist-vae/vq-vae/pre-trained/model/vqvae.Encoder/model_0_newest-0006.params", ctx=mx_context)
        decoder = gluon.nn.SymbolBlock.imports(home+"/wrk/vae/mnist-vae/vq-vae/pre-trained/model/vqvae.Decoder/model_0_newest-symbol.json",
                                               ['data'],
                                               home+"/wrk/vae/mnist-vae/vq-vae/pre-trained/model/vqvae.Decoder/model_0_newest-0006.params",
                                               ctx=mx_context)

    train_data = {}
    train_data["data"] = train_h5["data"]

    train_iter_pre_enc = mx.io.NDArrayIter(train_data, batch_size=batch_size)#train_data['data'].shape[0])
    for batch in train_iter_pre_enc:
        data = gluon.utils.split_and_load(batch.data[0], ctx_list=[mx_context], even_split=False)
        train_iter = encoder(data[0].as_in_context(mx_context))
        reconstructed = decoder(train_iter[0].as_in_context(mx_context))

        plt.subplot(1, 2, 1)
        plt.imshow(data[0][0].squeeze(axis=0).asnumpy())
        plt.title("Original")
        plt.axis("off")

        plt.subplot(1, 2, 2)
        plt.imshow(reconstructed[0].squeeze(axis=0).asnumpy())
        plt.title("Code")
        plt.axis("off")
        plt.show()

    test_data = {}
    test_data["data"] = test_h5["data"]

    test_iter_pre_enc = mx.io.NDArrayIter(test_data, batch_size=batch_size)


    test_iter = encoder(test_iter_pre_enc.as_in_context(mx.gpu()))

    return train_iter, test_iter


class ResidualBlock(gluon.nn.HybridBlock):
    def __init__(self):
        super(ResidualBlock, self).__init__()

    def hybrid_forward(self, F, x, *args, **kwargs):
        return 0


class MaskedConvB(gluon.nn.Conv2D):
    def __init__(self, channels, kernel_size, **kwargs):
        super().__init__(channels, kernel_size, **kwargs)

    def hybrid_forward(self, F, x, weight, bias=None):
        return 0


class MaskedConvA(gluon.nn.Conv2D):
    def __init__(self, channels, kernel_size, **kwargs):
        super().__init__(channels, kernel_size, **kwargs)

    def hybrid_forward(self, F, x, weight, bias=None):
        return 0


class PixelCNN(gluon.HybridBlock):
    def __init__(self):
        super(PixelCNN, self).__init__()
        with self.name_scope():
            pass

    def hybrid_forward(self, F, x):
        return


if __name__ == "__main__":
    #Needs to be same batch size as encoder
    train_data, test_data = get_data(200)
    pass