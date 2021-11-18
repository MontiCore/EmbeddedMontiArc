import warnings
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, gluon

if __name__ == "__main__":

    latent_dim = 8
    n_samples = 15

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        decoder = gluon.nn.SymbolBlock.imports("model/mnistvae.Decoder/model_0_newest-symbol.json", ['data'],
                                                        "model/mnistvae.Decoder/model_0_newest-0000.params", ctx=mx.cpu())
        #sample = mx.ndarray.random_normal(0,1,(1,8))
        #res = mx.ndarray.transpose(net(sample).squeeze(0)).asnumpy()
        #print(sample.asnumpy())

        zsamples = nd.array(np.random.randn(n_samples * n_samples, latent_dim))
        images = decoder(zsamples.as_in_context(mx.cpu())).asnumpy()

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
        plt.show()

