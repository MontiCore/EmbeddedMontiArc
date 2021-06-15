import mxnet as mx
from mxnet.gluon import HybridBlock, nn
from gluonnlp.model.seq2seq_encoder_decoder import Seq2SeqEncoder
from gluonnlp.model.bert import BERTEncoderCell

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

        if self._use_pooler:
            # Construct pooler
            self.pooler = self._get_pooler(units, 'pooler_')

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

        return tuple(outputs) if len(outputs) > 1 else outputs[0]

    def _encode_sequence(self, inputs, token_types, valid_length=None):
        # encoding
        outputs, additional_outputs = self.encoder(inputs, valid_length=valid_length)
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

    
class BERTEmbedding(HybridBlock):
    def __init__(self, 
            units=None,  
            max_length=None,
            dropout=None,
            weight_initializer=None, # not used at the moment
            layer_norm_eps=None,
            vocab_size=None, 
            token_type_vocab_size=None, 
            embed_size=None,
            embed_initializer=None,
            prefix=None,
            params=None
        ):
            super(BERTEmbedding, self).__init__(prefix=prefix, params=params)
            self._max_length = max_length
            self._units = units

            with self.name_scope():
                self.dropout_layer = nn.Dropout(rate=dropout)
                self.layer_norm = nn.LayerNorm(in_channels=units, epsilon=layer_norm_eps)
            self.word_embed = self._get_embed(
                vocab_size, embed_size,
                embed_initializer, 'word_embed_')
            self.token_type_embed = self._get_embed(
                token_type_vocab_size,
                embed_size, embed_initializer,
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