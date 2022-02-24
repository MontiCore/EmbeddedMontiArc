import logging
import sys
import warnings
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
from mxnet import nd, gluon

if __name__ == "__main__":
    # Get Encoder
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            encoder = gluon.nn.SymbolBlock.imports("model/vqvae.Encoder/model_0_newest-symbol.json", ['data'],
                                                        "model/vqvae.Encoder/model_0_newest-0015.params", ctx=mx.cpu())
        except:
            logging.error(" Failed while trying to load the encoder model")

    # Get Decoder
        try:
            decoder = gluon.nn.SymbolBlock.imports("model/vqvae.Decoder/model_0_newest-symbol.json", ['data'],
                                                        "model/vqvae.Decoder/model_0_newest-0015.params", ctx=mx.cpu())
        except:
            logging.error(" Failed while trying to load the decoder model")

    # Get trained codebook
    codebook = gluon.ParameterDict()
    codebook.get("vectorquantize0_embeddings")
    codebook.load(filename="model/vqvae.Encoder/model_0-vectorquantize0-0000.params", ctx=mx.cpu())
    try:
        codebook.get("vectorquantize0_embeddings").data()
    except:
        logging.error("Failed while trying to load the codebook component")
        sys.exit(1)



