import argparse
import logging
import sys
import warnings

#import cv2
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image, ImageOps
from mxnet import nd, gluon

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ctx", action="store", dest="context", default="cpu")
    parser.add_argument("--image", action="store", dest="image", default="resources/test_image_1.png")
    parser.add_argument("--digit", action="store", dest="digit", default=1)
    args = parser.parse_args()
    if args.context == "cpu":
        mx_ctx = mx.cpu()
    elif args.context == "gpu":
        mx_ctx = mx.gpu()
    else:
        logging.error("Argument Error: ctx can only be 'cpu' or 'gpu'.")
        sys.exit(1)

    classes = [0,1,2,3,4,5,6,7,8,9]
    if not int(args.digit) in classes:
        logging.error("Argument Error: Argument digit was not provided with a digit.")

    try:
        #img = cv2.cvtColor(cv2.imread(args.image), cv2.COLOR_BGR2GRAY)
        img = np.asarray(ImageOps.grayscale(Image.open((args.image))).resize((28,28))).reshape((1,1,28,28))
    except:
        logging.error("Argument Error: No image found in given path")

    #input = mx.nd.array(np.reshape(cv2.resize(img, dsize=(28, 28), interpolation=cv2.INTER_CUBIC), (1, 1, 28, 28)),ctx=mx_ctx) / 255
    input = mx.nd.array(img,ctx=mx_ctx) / 255

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")

        # Get Encoder
        #try:
        encoder = gluon.nn.SymbolBlock.imports("model/mnist.Encoder/model_0_newest-symbol.json",
                                               ['data'],
                                               "model/mnist.Encoder/model_0_newest-0000.params", ctx=mx_ctx)
        #except:
        #    logging.error(" Failed while trying to load the encoder model")
        #    sys.exit(1)

        # Get Decoder
        try:
            decoder = gluon.nn.SymbolBlock.imports("model/mnist.Decoder/model_0_newest-symbol.json", ['data'],
                                                        "model/mnist.Decoder/model_0_newest-0000.params", ctx=mx_ctx)
        except:
            print(" Failed while trying to load the decoder model")
            sys.exit(1)

        # Get LeNet
        try:
            leNet = gluon.nn.SymbolBlock.imports("model/mnist.LeNetNetwork/model_0_newest-symbol.json", ['data'],
                                                        "model/mnist.LeNetNetwork/model_0_newest-0000.params", ctx=mx_ctx)
        except:
            print(" Failed while trying to load the decoder model")
            sys.exit(1)

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            encoded, _, _ = encoder(input.as_in_context(mx_ctx))
            decoded = decoder(encoded.as_in_context(mx_ctx))
            probs = leNet(decoded.as_in_context(mx_ctx))[0]

        plt.imsave(f"reconstructed_{int(args.digit)}.png",decoded[0][0].asnumpy())

        predicited_class = np.argmax(probs.asnumpy())

        if predicited_class == int(args.digit):
            print("Successfully predicted reconstruction of digit: " + str(predicited_class))
        else:
            logging.error(f"Test failed \n Predicited digit: {predicited_class} \n Expected digit: {args.digit}")
            sys.exit(1)







