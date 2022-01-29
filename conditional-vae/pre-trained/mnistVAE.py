import warnings
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, gluon
import argparse

from numpy.core.fromnumeric import shape

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--number", action="store", dest="number", default=7)
    args = parser.parse_args()

    latent_dim = 2
    n_samples = 10

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        decoder = gluon.nn.SymbolBlock.imports("model/cvae.Decoder/model_0_newest-symbol.json", ['data0','data1'],
                                                        "model/cvae.Decoder/model_0_newest-0015.params", ctx=mx.cpu())
        #sample = mx.ndarray.random_normal(0,1,(1,8))
        #res = mx.ndarray.transpose(net(sample).squeeze(0)).asnumpy()
        #print(sample.asnumpy())
        y = np.linspace(-3, 3, n_samples)
        x = np.linspace(3, 3, n_samples)
        Y,X = np.meshgrid(y,x)
        z = nd.array(np.array([Y.flatten(),X.flatten()]).T)
        label = nd.array(np.array([
                         np.full((10),0),
                         np.full((10),1),
                         np.full((10),2),
                         np.full((10),3),
                         np.full((10),4),
                         np.full((10),5),
                         np.full((10),6),
                         np.full((10),7),
                         np.full((10),8),
                         np.full((10),9)]))

        oh = mx.ndarray.reshape(mx.ndarray.one_hot(label,10),shape=(100,10))
        #boh = mx.ndarray.broadcast_axis(oh,axis=0,size=n_samples)
        

        zsamples = nd.array(np.random.randn(n_samples * n_samples, latent_dim))
        images = decoder(z.as_in_context(mx.cpu()),oh.as_in_context(mx.cpu())).asnumpy()

        canvas = np.empty((28 * n_samples, 28 * n_samples))
        for i, img in enumerate(images):
            x = i // n_samples
            y = i % n_samples
            canvas[(n_samples - y - 1) * 28:(n_samples - y) * 28, x * 28:(x + 1) * 28] = img.reshape(28, 28)
        plt.figure(figsize=(10, 10))
        plt.imshow(canvas, origin="upper", cmap='Greys')
        plt.axis('off')
        plt.tight_layout()
        plt.savefig('generated_digits')
        plt.close()

