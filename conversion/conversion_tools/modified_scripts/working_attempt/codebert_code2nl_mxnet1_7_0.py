from .convert_codebert_transformers_mxnet1_7_0 import convert_huggingface_model
from gluonnlp.model.transformer import TransformerDecoder
from mxnet.gluon.block import HybridBlock
import mxnet.gluon.nn as nn
import mxnet as mx

a = TransformerDecoder()

class Seq2Seq(HybridBlock):
    def __init__(
            self, encoder, decoder, 
            hidden_size=None, beam_size=None, 
            max_length=None, sos_id=None, 
            eos_id=None, prefix=None, params=None
        ):
            super().__init__(prefix=prefix, params=params)
            self.encoder = encoder
            self.decoder = decoder
            self.bias = mx.ndarray.linalg.extracttrian(mx.nd.ones((2048,2048)))
            self.dense = nn.Dense(hidden_size, hidden_size)
            # tie the lm_head and word_embed params together not sure if this is the correct way to do it
            self.lm_head = nn.Dense(hidden_size, hidden_size, use_bias=False, params=encoder.word_embed.params)
            self.lsm = mx.npx.log_softmax(axis=-1) # axis = -1 the same as dim = -1?
            self.beam_size=beam_size
            self.max_length=max_length
            self.sos_id=sos_id
            self.eos_id=eos_id