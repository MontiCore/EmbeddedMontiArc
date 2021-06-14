__all__ = ['BERTModel', 'RoBERTaModel', 'BERTEncoder', 'BERTClassifier',
           'RoBERTaClassifier', 'bert_12_768_12', 'bert_24_1024_16',
           'ernie_12_768_12', 'roberta_12_768_12', 'roberta_24_1024_16',
           'DistilBERTModel', 'distilbert_6_768_12']

import os

import mxnet as mx
from mxnet.gluon import HybridBlock, nn
from mxnet.gluon.model_zoo import model_store

from ..base import get_home_dir
from .block import GELU
from .seq2seq_encoder_decoder import Seq2SeqEncoder
from .transformer import PositionwiseFFN
from .utils import _load_pretrained_params, _load_vocab

class BERTEncoder(HybridBlock, Seq2SeqEncoder):
    def __init__(self, *, num_layers=2, units=512, hidden_size=2048,
                 max_length=50, num_heads=4, dropout=0.0,
                 output_attention=False, output_all_encodings=False, weight_initializer=None,
                 bias_initializer='zeros', prefix=None, params=None, activation='gelu',
                 layer_norm_eps=1e-12):
        super().__init__(prefix=prefix, params=params)
        assert units % num_heads == 0,\
            'In BERTEncoder, The units should be divided exactly ' \
            'by the number of heads. Received units={}, num_heads={}' \
            .format(units, num_heads)
        self._max_length = max_length
        self._units = units
        self._output_attention = output_attention
        self._output_all_encodings = output_all_encodings
        self._dropout = dropout
        self._layer_norm_eps = layer_norm_eps

        with self.name_scope():
            # if dropout:
            #     self.dropout_layer = nn.Dropout(rate=dropout)
            # self.layer_norm = nn.LayerNorm(in_channels=units, epsilon=self._layer_norm_eps)
            # self.position_weight = self.params.get('position_weight', shape=(max_length, units),
            #                                        init=weight_initializer)
            self.transformer_cells = nn.HybridSequential()
            for i in range(num_layers):
                cell = BERTEncoderCell(
                    units=units, hidden_size=hidden_size, num_heads=num_heads,
                    weight_initializer=weight_initializer,
                    bias_initializer=bias_initializer, dropout=dropout,
                    output_attention=output_attention, prefix='transformer%d_' % i,
                    activation=activation, layer_norm_eps=layer_norm_eps)
                self.transformer_cells.add(cell)

    def __call__(self, inputs, states=None, valid_length=None):  # pylint: disable=arguments-differ
        return super().__call__(inputs, states, valid_length)

    def hybrid_forward(self, F, inputs, states=None, valid_length=None, position_weight=None):
        # axis 0 is for length
        steps = F.contrib.arange_like(inputs, axis=0)
        if valid_length is not None:
            zeros = F.zeros_like(steps)
            # valid_length for attention, shape = (batch_size, seq_length)
            attn_valid_len = F.broadcast_add(F.reshape(valid_length, shape=(-1, 1)),
                                             F.reshape(zeros, shape=(1, -1)))
            attn_valid_len = F.cast(attn_valid_len, dtype='int32')
            if states is None:
                states = [attn_valid_len]
            else:
                states.append(attn_valid_len)
        else:
            attn_valid_len = None

        if states is None:
            states = [steps]
        else:
            states.append(steps)

        # # positional encoding
        # positional_embed = F.Embedding(steps, position_weight, self._max_length, self._units)
        # inputs = F.broadcast_add(inputs, F.expand_dims(positional_embed, axis=1))

        # if self._dropout:
        #     inputs = self.dropout_layer(inputs)
        # inputs = self.layer_norm(inputs)
        outputs = inputs

        all_encodings_outputs = []
        additional_outputs = []
        for cell in self.transformer_cells:
            outputs, attention_weights = cell(inputs, attn_valid_len)
            inputs = outputs
            if self._output_all_encodings:
                if valid_length is not None:
                    outputs = F.SequenceMask(outputs, sequence_length=valid_length,
                                             use_sequence_length=True, axis=0)
                all_encodings_outputs.append(outputs)

            if self._output_attention:
                additional_outputs.append(attention_weights)

        if valid_length is not None and not self._output_all_encodings:
            # if self._output_all_encodings, SequenceMask is already applied above
            outputs = F.SequenceMask(outputs, sequence_length=valid_length,
                                     use_sequence_length=True, axis=0)

        if self._output_all_encodings:
            return all_encodings_outputs, additional_outputs
        return outputs, additional_outputs

class BERTModel(HybridBlock):
    def __init__(self, encoder, vocab_size=None, token_type_vocab_size=None, units=None,
                 embed_size=None, embed_initializer=None,
                 word_embed=None, token_type_embed=None, use_pooler=True, use_decoder=True,
                 use_classifier=True, use_token_type_embed=True, prefix=None, params=None):
        super().__init__(prefix=prefix, params=params)
        self._use_decoder = use_decoder
        self._use_classifier = use_classifier
        self._use_pooler = use_pooler
        self._use_token_type_embed = use_token_type_embed
        self._units = units
        self.encoder = encoder
        # Construct word embedding
        self.word_embed = self._get_embed(word_embed, vocab_size, embed_size,
                                          embed_initializer, 'word_embed_')
        # Construct token type embedding
        if use_token_type_embed:
            self.token_type_embed = self._get_embed(token_type_embed, token_type_vocab_size,
                                                    embed_size, embed_initializer,
                                                    'token_type_embed_')
        if self._use_pooler:
            # Construct pooler
            self.pooler = self._get_pooler(units, 'pooler_')
            if self._use_classifier:
                # Construct classifier for next sentence predicition
                self.classifier = self._get_classifier('cls_')
        else:
            assert not use_classifier, 'Cannot use classifier if use_pooler is False'
        if self._use_decoder:
            # Construct decoder for masked language model
            self.decoder = self._get_decoder(units, vocab_size, self.word_embed[0], 'decoder_')

    def _get_classifier(self, prefix):
        """ Construct a decoder for the next sentence prediction task """
        with self.name_scope():
            classifier = nn.Dense(2, prefix=prefix)
        return classifier

    def _get_decoder(self, units, vocab_size, embed, prefix):
        """ Construct a decoder for the masked language model task """
        with self.name_scope():
            decoder = nn.HybridSequential(prefix=prefix)
            decoder.add(nn.Dense(units, flatten=False))
            decoder.add(GELU())
            decoder.add(nn.LayerNorm(in_channels=units, epsilon=self.encoder._layer_norm_eps))
            decoder.add(nn.Dense(vocab_size, flatten=False, params=embed.collect_params()))
        assert decoder[3].weight == list(embed.collect_params().values())[0], \
            'The weights of word embedding are not tied with those of decoder'
        return decoder

    def _get_embed(self, embed, vocab_size, embed_size, initializer, prefix):
        """ Construct an embedding block. """
        if embed is None:
            assert embed_size is not None, '"embed_size" cannot be None if "word_embed" or ' \
                                           'token_type_embed is not given.'
            with self.name_scope():
                embed = nn.HybridSequential(prefix=prefix)
                with embed.name_scope():
                    embed.add(nn.Embedding(input_dim=vocab_size, output_dim=embed_size,
                                           weight_initializer=initializer))
        assert isinstance(embed, HybridBlock)
        return embed

    def _get_pooler(self, units, prefix):
        """ Construct pooler.

        The pooler slices and projects the hidden output of first token
        in the sequence for segment level classification.

        """
        with self.name_scope():
            pooler = nn.Dense(units=units, flatten=False, activation='tanh',
                              prefix=prefix)
        return pooler

    def __call__(self, inputs, token_types, valid_length=None, masked_positions=None):
        # pylint: disable=dangerous-default-value, arguments-differ
        """Generate the representation given the inputs.

        This is used in training or fine-tuning a BERT model.
        """
        return super().__call__(inputs, token_types, valid_length, masked_positions)

    def hybrid_forward(self, F, inputs, token_types, valid_length=None, masked_positions=None):
        # pylint: disable=arguments-differ
        """Generate the representation given the inputs.

        This is used in training or fine-tuning a BERT model.
        """
        outputs = []
        seq_out, attention_out = self._encode_sequence(inputs, token_types, valid_length)
        outputs.append(seq_out)

        if self.encoder._output_all_encodings:
            assert isinstance(seq_out, list)
            output = seq_out[-1]
        else:
            output = seq_out

        if attention_out:
            outputs.append(attention_out)

        if self._use_pooler:
            pooled_out = self._apply_pooling(output)
            outputs.append(pooled_out)
            if self._use_classifier:
                next_sentence_classifier_out = self.classifier(pooled_out)
                outputs.append(next_sentence_classifier_out)
        if self._use_decoder:
            assert masked_positions is not None, \
                'masked_positions tensor is required for decoding masked language model'
            decoder_out = self._decode(F, output, masked_positions)
            outputs.append(decoder_out)
        return tuple(outputs) if len(outputs) > 1 else outputs[0]


    def _encode_sequence(self, inputs, token_types, valid_length=None):
        """Generate the representation given the input sequences.

        This is used for pre-training or fine-tuning a BERT model.
        """
        # embedding
        embedding = self.word_embed(inputs)
        if self._use_token_type_embed:
            type_embedding = self.token_type_embed(token_types)
            embedding = embedding + type_embedding
        # (batch, seq_len, C) -> (seq_len, batch, C)
        embedding = embedding.transpose((1, 0, 2))
        # encoding
        outputs, additional_outputs = self.encoder(embedding, valid_length=valid_length)
        # (seq_len, batch, C) -> (batch, seq_len, C)
        if isinstance(outputs, (list, tuple)):
            outputs = [o.transpose((1, 0, 2)) for o in outputs]
        else:
            outputs = outputs.transpose((1, 0, 2))
        return outputs, additional_outputs

    def _apply_pooling(self, sequence):
        """Generate the representation given the inputs.

        This is used for pre-training or fine-tuning a BERT model.
        """
        outputs = sequence.slice(begin=(0, 0, 0), end=(None, 1, None))
        outputs = outputs.reshape(shape=(-1, self._units))
        return self.pooler(outputs)

    def _decode(self, F, sequence, masked_positions):
        """Generate unnormalized prediction for the masked language model task.

        This is only used for pre-training the BERT model.

        Inputs:
            - **sequence**: input tensor of sequence encodings.
              Shape (batch_size, seq_length, units).
            - **masked_positions**: input tensor of position of tokens for masked LM decoding.
              Shape (batch_size, num_masked_positions). For each sample in the batch, the values
              in this tensor must not be out of bound considering the length of the sequence.

        Outputs:
            - **masked_lm_outputs**: output tensor of token predictions for target masked_positions.
                Shape (batch_size, num_masked_positions, vocab_size).
        """
        masked_positions = masked_positions.astype('int32')
        mask_shape = masked_positions.shape_array()
        num_masked_positions = mask_shape.slice(begin=(1,), end=(2,)).astype('int32')
        idx_arange = F.contrib.arange_like(masked_positions.reshape((-1, )), axis=0)
        batch_idx = F.broadcast_div(idx_arange, num_masked_positions)
        # batch_idx_1d =        [0,0,0,1,1,1,2,2,2...]
        # masked_positions_1d = [1,2,4,0,3,4,2,3,5...]
        batch_idx_1d = batch_idx.reshape((1, -1))
        masked_positions_1d = masked_positions.reshape((1, -1))
        position_idx = F.concat(batch_idx_1d, masked_positions_1d, dim=0)
        encoded = F.gather_nd(sequence, position_idx)
        encoded = encoded.reshape_like(masked_positions, lhs_begin=-2, lhs_end=-1, rhs_begin=0)
        decoded = self.decoder(encoded)
        return decoded


# (self, *, num_layers=2, units=512, hidden_size=2048,
#                  max_length=50, num_heads=4, dropout=0.0,
#                  output_attention=False, output_all_encodings=False, weight_initializer=None,
#                  bias_initializer='zeros', prefix=None, params=None, activation='gelu',
#                  layer_norm_eps=1e-12):

class BERTEmbedding(HybridBlock):
    def __init__(self, 
            units=None,  
            max_length=None,
            dropout=None,
            initializer=None,
            layer_norm_eps=None,
            vocab_size=None, 
            token_type_vocab_size=None, 
            embed_size=None
        ):

            self._max_length = max_length
            self._units = units

            with self.name_scope():
                self.dropout_layer = nn.Dropout(rate=dropout)
                self.layer_norm = nn.LayerNorm(in_channels=units, epsilon=layer_norm_eps)
            self.word_embed = self._get_embed(
                vocab_size, embed_size,
                initializer, 'word_embed_')
            self.token_type_embed = self._get_embed(
                token_type_vocab_size,
                embed_size, initializer,
                'token_type_embed_')

    def hybrid_forward(self, F, inputs, token_types, position_weight=None):
        # embedding
        embedding = self.word_embed(inputs)
        type_embedding = self.token_type_embed(token_types)
        embedding = embedding + type_embedding
        # (batch, seq_len, C) -> (seq_len, batch, C)
        embedding = embedding.transpose((1, 0, 2))

        steps = F.contrib.arange_like(embedding, axis=0)
        positional_embed = F.Embedding(steps, position_weight, self._max_length, self._units)
        embedding = F.broadcast_add(embedding, F.expand_dims(positional_embed, axis=1))

        embedding = self.dropout_layer(embedding)
        embedding = self.layer_norm(embedding)

        return embedding
    
    def _get_embed(self, vocab_size, embed_size, initializer, prefix):
        """ Construct an embedding block. """
            with self.name_scope():
                embed = nn.HybridSequential(prefix=prefix)
                with embed.name_scope():
                    embed.add(nn.Embedding(input_dim=vocab_size, output_dim=embed_size,
                                           weight_initializer=initializer))
        assert isinstance(embed, HybridBlock)
        return embed