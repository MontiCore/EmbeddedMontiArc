import mxnet as mx
import numpy as np
from mxnet.gluon import HybridBlock, nn
from gluonnlp.model.seq2seq_encoder_decoder import Seq2SeqDecoder, Seq2SeqEncoder
from gluonnlp.model.bert import BERTEncoderCell
from mxnet.gluon.block import initializer
from gluonnlp.model.transformer import TransformerDecoderCell, _BaseTransformerDecoder

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

    def hybrid_forward(self, F, inputs, states=None, valid_length=None):
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
        self.encoder = encoder

    def __call__(self, inputs, valid_length=None):
        return super().__call__(inputs, valid_length)

    def hybrid_forward(self, F, inputs, valid_length=None):
        # remove single dim entries from the valid_length input, needed for compatibility with EMADL LoadNetwork layer
        # doesnt do anything to already flat arrays
        valid_length = mx.symbol.squeeze(valid_length) # [[5], [6], ...] -> [5, 6, ....]
        outputs = []
        seq_out, attention_out = self._encode_sequence(inputs, valid_length)
        outputs.append(seq_out)

        if self.encoder._output_all_encodings:
            assert isinstance(seq_out, list)
            output = seq_out[-1]
        else:
            output = seq_out

        if attention_out:
            outputs.append(attention_out)

        return tuple(outputs) if len(outputs) > 1 else outputs[0]

    def _encode_sequence(self, inputs, valid_length=None):
        # encoding
        outputs, additional_outputs = self.encoder(inputs, valid_length=valid_length)
        # (seq_len, batch, C) -> (batch, seq_len, C)
        if isinstance(outputs, (list, tuple)):
            outputs = [o.transpose((1, 0, 2)) for o in outputs]
        else:
            outputs = outputs.transpose((1, 0, 2))
        return outputs, additional_outputs

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
            super().__init__(prefix=prefix, params=params)
            self._max_length = max_length
            self._units = units

            with self.name_scope():
                self.dropout_layer = nn.Dropout(rate=dropout)
                self.layer_norm = nn.LayerNorm(in_channels=units, epsilon=layer_norm_eps)
                self.position_weight = self.params.get(
                    'position_weight', shape=(max_length, units),
                    init=weight_initializer)
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
        with self.name_scope():
            embed = nn.HybridSequential(prefix=prefix)
            with embed.name_scope():
                embed.add(nn.Embedding(input_dim=vocab_size, output_dim=embed_size,
                                        weight_initializer=initializer))
        assert isinstance(embed, HybridBlock)
        return embed

class Seq2Seq(HybridBlock):
    def __init__(
            self, embedding, encoder, decoder,
            vocab_size=None, hidden_size=None, beam_size=None, 
            max_length=None, sos_id=None, 
            eos_id=None, prefix=None, params=None
        ):
            super().__init__(prefix=prefix, params=params)
            self.embedding = embedding
            self.encoder = encoder
            self.decoder = decoder
            self.bias = mx.ndarray.linalg.extracttrian(mx.nd.ones((2048,2048)))
            self.hidden_size = hidden_size
            self.dense = nn.Dense(hidden_size, in_units=hidden_size, activation='tanh')
            # tie the lm_head and word_embed params together not sure if this is the correct way to do it
            # TODO maybe embedding params object is in dictionary and this is referencing all the params of the embedding layers?
            self.lm_head = nn.Dense(vocab_size, in_units=hidden_size, use_bias=False)
            self.beam_size=beam_size
            self.max_length=max_length
            self.sos_id=sos_id
            self.eos_id=eos_id
    
    def initialize(self, init=initializer.Uniform(), ctx=None, verbose=False):
        # only initialize new params, not the ones from pretrained model
        # param._data none check taken from Parameter initialize method
        if verbose:
            init.set_verbosity(verbose=verbose)
        for _, param in self.collect_params().items():
            if param._data is None:
                param.initialize(init, ctx, force_reinit=False)
        
        # self.collect_params().initialize(init, ctx, verbose, force_reinit=False)
        # tie weights of lm head and embedding layer, done in torch script too
        embed = self.embedding.collect_params()['bertembedding0_word_embed_embedding0_weight'].data(ctx=ctx[0])
        self.lm_head.collect_params()['dense1_weight'].set_data(embed)

    def valid_length_to_mask(self, input_ids, valid_length):
        input_mask = mx.nd.zeros_like(input_ids)
        np_valid_len = valid_length.asnumpy().astype(np.int32)
        for i in range(len(input_ids)):
            input_mask[i][0:np_valid_len[i]] = 1
        return input_mask

    def forward(self, source_ids=None, source_valid_length=None, target_ids=None, target_valid_length=None):   
        source_token_types = mx.nd.zeros_like(source_ids)
        embed_output = self.embedding(source_ids, source_token_types)
        encoder_output = self.encoder(embed_output, source_valid_length) # we don't permute like in the code2nl model, that okay? TODO shape 8x256x768
        #encoder_output = outputs[0].permute([1,0,2]).contiguous() not sure how to do
        if target_ids is not None:
            target_token_types = mx.nd.zeros_like(target_ids)
            # could try subclassing the Decoder and change the hybrid_forward function.
            # transpose so we have (batch size, target seq length, embed dims)
            tgt_embeddings = self.embedding(target_ids, target_token_types).transpose((1, 0, 2))
            states = self.decoder.init_state_from_encoder(encoder_output, source_valid_length)
            # TODO do we need to set position weight to something for decoder?
            # target_valid_length fails when it is a named parameter for some reason, so we just pass as the third
            out, _, _ = self.decoder(tgt_embeddings, states, target_valid_length)
            # if 8 batch size, 128 target length, 765 embed size, 50265 vocab size
            # (8,128,768) -> transpose -> (128,8,768) -> reshape -> (1024,768)
            hidden_states = self.dense(out.transpose((1, 0, 2)).reshape(-1, self.hidden_size))
            # TODO do we need to reshape back to original dimensions here?
            # (1024,768) -> (1024, 50265)
            lm_logits = self.lm_head(hidden_states)
            # reshape back to original dimensions -> (128, 8, 50265) -> (8, 128, 50265)
            lm_logits = lm_logits.reshape(self.max_length, -1, lm_logits.shape[-1]).transpose((1, 0, 2))
            return lm_logits
        else:
            #Predict
            input_ctx = source_ids.ctx
            source_mask = self.valid_length_to_mask(source_ids, source_valid_length)
            preds=[]
            output_probs = []       
            zero = mx.nd.zeros(1, dtype='int64', ctx=input_ctx)
            # iterate accross sequences in batch
            for i in range(source_ids.shape[0]):
                # shape is (batch_size, seq_len, embed_size), we want the ith sequence
                context = encoder_output[i:i+1,:]
                context_valid_len = source_valid_length[i:i+1]
                context_mask = source_mask[i:i+1,:]
                beam = Beam(self.beam_size, self.sos_id, self.eos_id, input_ctx)
                input_ids = beam.getCurrentState()
                input_token_types = mx.nd.zeros_like(input_ids)
                input_valid_length = mx.nd.ones(input_ids.shape[0], ctx=input_ctx) # TODO should we use anything other than ones here? only if the beam adds padding
                context = context.tile((self.beam_size, 1, 1))
                context_mask = context_mask.tile((self.beam_size, 1)) # TODO unused, delete
                context_valid_len = context_valid_len.tile((self.beam_size))
                states = self.decoder.init_state_from_encoder(context, context_valid_len)
                for i in range(self.max_length): 
                    if beam.done():
                        break
                    # still not sure how important this is, we cant really use it in our decoder?
                    # attn_mask=-1e4 *(1-self.bias[:input_ids.shape[1],:input_ids.shape[1]])
                    tgt_embeddings = self.embedding(input_ids, input_token_types).transpose((1, 0, 2))
                    out, _, _ = self.decoder(tgt_embeddings, states, input_valid_length)
                     # combine first two dims to pass through dense layer
                    hidden_states = self.dense(out.reshape(-1, self.hidden_size))
                    # recreate first two dims and take last word in sequence
                    # we dont have to transpose here because our output is already the desired format of (beam_size, seq_len, embed_size)
                    hidden_states = out.reshape(self.beam_size, -1, self.hidden_size)[:,-1,:] 
                    lm_logits = self.lm_head(hidden_states)
                    out = mx.nd.log_softmax(lm_logits, axis=-1)
                    output_probs.append(out.copyto(mx.cpu()).detach())
                    beam.advance(out)
                    input_ids.take(beam.getCurrentOrigin(), 0).copyto(input_ids)
                    input_ids = mx.nd.concat(input_ids, beam.getCurrentState(), dim=-1)
                    input_token_types = mx.nd.zeros_like(input_ids)
                    input_valid_length += 1
                
                hyp = beam.getHyp(beam.getFinal()) #hyp is (beam_size, target_seq_len
                pred = beam.buildTargetTokens(hyp)[:self.beam_size]
                # pred = [torch.cat([x.view(-1) for x in p]+[zero]*(self.max_length-len(p))).view(1,-1) for p in pred]
                pred = [mx.nd.concat(*([x.reshape(-1) for x in p]+[zero]*(self.max_length-len(p))), dim=-1).reshape(1,-1) for p in pred]
                preds.append(mx.nd.concat(*pred, dim=0).expand_dims(0))
                
            preds = mx.nd.concat(*preds, dim=0)
            return preds, output_probs

class Beam(object):
    def __init__(self, size, sos, eos, ctx):
        self.size = size
        # The score for each translation on the beam.
        self.scores = mx.nd.zeros(size, ctx=ctx)
        # The backpointers at each time-step.
        self.prevKs = []
        # The outputs at each time-step.
        self.nextYs = [mx.nd.zeros(size, dtype='int64', ctx=ctx)]
        self.nextYs[0][0] = sos
        # Has EOS topped the beam yet.
        self._eos = eos
        self.eosTop = False
        # Time and k pair for finished.
        self.finished = []
        self.ctx = ctx

    def getCurrentState(self):
        "Get the outputs for the current timestep."
        batch = mx.nd.array(self.nextYs[-1], dtype='int64', ctx=self.ctx).reshape(-1, 1)
        return batch

    def getCurrentOrigin(self):
        "Get the backpointers for the current timestep."
        return self.prevKs[-1]

    def advance(self, wordLk):
        """
        Given prob over words for every last beam `wordLk` and attention
        `attnOut`: Compute and update the beam search.

        Parameters:

        * `wordLk`- probs of advancing from the last step (K x words)
        * `attnOut`- attention at the last step

        Returns: True if beam search is complete.
        """
        numWords = wordLk.shape[1]

        # Sum the previous scores.
        if len(self.prevKs) > 0:
            beamLk = wordLk + self.scores.expand_dims(1).broadcast_like(wordLk)

            # Don't let EOS have children.
            for i in range(self.nextYs[-1].shape[0]):
                if self.nextYs[-1][i] == self._eos:
                    beamLk[i] = -1e20
        else:
            beamLk = wordLk[0]
        flatBeamLk = beamLk.reshape(-1)
        bestScores, bestScoresId = flatBeamLk.topk(k=self.size, axis=0, ret_typ='both')

        self.scores = bestScores

        # bestScoresId is flattened beam x word array, so calculate which
        # word and beam each score came from
        prevK = (bestScoresId / numWords).floor() # TODO replaced // with / and floor, they should be the same? because mxnet doesnt have //
        self.prevKs.append(prevK)
        self.nextYs.append((bestScoresId - prevK * numWords))


        for i in range(self.nextYs[-1].shape[0]):
            if self.nextYs[-1][i] == self._eos:
                s = self.scores[i]
                self.finished.append((s, len(self.nextYs) - 1, i))

        # End condition is when top-of-beam is EOS and no global score.
        if self.nextYs[-1][0] == self._eos:
            self.eosTop = True

    def done(self):
        return self.eosTop and len(self.finished) >=self.size

    def getFinal(self):
        if len(self.finished) == 0:
            self.finished.append((self.scores[0], len(self.nextYs) - 1, 0))
        self.finished.sort(key=lambda a: -a[0])
        if len(self.finished) != self.size:
            unfinished=[]
            for i in range(self.nextYs[-1].shape[0]):
                if self.nextYs[-1][i] != self._eos:
                    s = self.scores[i]
                    unfinished.append((s, len(self.nextYs) - 1, i)) 
            unfinished.sort(key=lambda a: -a[0])
            self.finished+=unfinished[:self.size-len(self.finished)]
        return self.finished[:self.size]

    def getHyp(self, beam_res):
        """
        Walk back to construct the full hypothesis.
        """
        hyps=[]
        for _,timestep, k in beam_res:
            hyp = []
            for j in range(len(self.prevKs[:timestep]) - 1, -1, -1):
                hyp.append(self.nextYs[j+1][k])
                k = self.prevKs[j][k]
            hyps.append(hyp[::-1])
        return hyps
    
    def buildTargetTokens(self, preds):
        sentence=[]
        for pred in preds:
            tokens = []
            for tok in pred:
                if tok==self._eos:
                    break
                tokens.append(tok)
            sentence.append(tokens)
        return sentence

class _BaseTransformerDecoder(HybridBlock):
    def __init__(self, attention_cell='multi_head', num_layers=2, units=128, hidden_size=2048,
                 max_length=50, num_heads=4, scaled=True, scale_embed=True, norm_inputs=True,
                 dropout=0.0, use_residual=True, output_attention=False, weight_initializer=None,
                 bias_initializer='zeros', prefix=None, params=None):
        super().__init__(prefix=prefix, params=params)
        assert units % num_heads == 0, 'In TransformerDecoder, the units should be divided ' \
                                       'exactly by the number of heads. Received units={}, ' \
                                       'num_heads={}'.format(units, num_heads)
        self._num_layers = num_layers
        self._units = units
        self._hidden_size = hidden_size
        self._num_states = num_heads
        self._max_length = max_length
        self._dropout = dropout
        self._use_residual = use_residual
        self._output_attention = output_attention
        self._scaled = scaled # scaled should be true because we want to compute scaled dot product attention
        self._scale_embed = scale_embed
        self._norm_inputs = norm_inputs
        with self.name_scope():
            if dropout:
                self.dropout_layer = nn.Dropout(rate=dropout)
            if self._norm_inputs:
                self.layer_norm = nn.LayerNorm()
            # encoding = _position_encoding_init(max_length, units)
            # self.position_weight = self.params.get_constant('const', encoding.astype(np.float32))
            self.transformer_cells = nn.HybridSequential()
            for i in range(num_layers):
                self.transformer_cells.add(
                    TransformerDecoderCell(units=units, hidden_size=hidden_size,
                                           num_heads=num_heads, attention_cell=attention_cell,
                                           weight_initializer=weight_initializer,
                                           bias_initializer=bias_initializer, dropout=dropout,
                                           scaled=scaled, use_residual=use_residual,
                                           output_attention=output_attention,
                                           prefix='transformer%d_' % i))

    def init_state_from_encoder(self, encoder_outputs, encoder_valid_length=None):
        """Initialize the state from the encoder outputs.

        Parameters
        ----------
        encoder_outputs : list
        encoder_valid_length : NDArray or None

        Returns
        -------
        decoder_states : list
            The decoder states, includes:

            - mem_value : NDArray
            - mem_masks : NDArray or None
        """
        mem_value = encoder_outputs
        decoder_states = [mem_value]
        mem_length = mem_value.shape[1]
        if encoder_valid_length is not None:
            dtype = encoder_valid_length.dtype
            ctx = encoder_valid_length.context
            mem_masks = mx.nd.broadcast_lesser( # could be the attn masks?
                mx.nd.arange(mem_length, ctx=ctx, dtype=dtype).reshape((1, -1)),
                encoder_valid_length.reshape((-1, 1)))
            decoder_states.append(mem_masks)
        else:
            decoder_states.append(None)
        return decoder_states

    def hybrid_forward(self, F, inputs, states, valid_length=None, position_weight=None):
        #pylint: disable=arguments-differ
        """Decode the decoder inputs. This function is only used for training.

        Parameters
        ----------
        inputs : NDArray, Shape (batch_size, length, C_in)
        states : list of NDArrays or None
            Initial states. The list of decoder states
        valid_length : NDArray or None
            Valid lengths of each sequence. This is usually used when part of sequence has
            been padded. Shape (batch_size,)

        Returns
        -------
        output : NDArray, Shape (batch_size, length, C_out)
        states : list
            The decoder states:
            - mem_value : NDArray
            - mem_masks : NDArray or None
        additional_outputs : list of list
            Either be an empty list or contains the attention weights in this step.
            The attention weights will have shape (batch_size, length, mem_length) or
            (batch_size, num_heads, length, mem_length)
        """

        length_array = F.contrib.arange_like(inputs, axis=1)
        mask = F.broadcast_lesser_equal(length_array.reshape((1, -1)),
                                        length_array.reshape((-1, 1)))
        if valid_length is not None:
            batch_mask = F.broadcast_lesser(length_array.reshape((1, -1)),
                                            valid_length.reshape((-1, 1)))
            batch_mask = F.expand_dims(batch_mask, -1)
            mask = F.broadcast_mul(batch_mask, F.expand_dims(mask, 0))
        else:
            mask = F.expand_dims(mask, axis=0)
            mask = F.broadcast_like(mask, inputs, lhs_axes=(0, ), rhs_axes=(0, ))

        mem_value, mem_mask = states
        if mem_mask is not None:
            mem_mask = F.expand_dims(mem_mask, axis=1)
            mem_mask = F.broadcast_like(mem_mask, inputs, lhs_axes=(1, ), rhs_axes=(1, ))

        # if self._scale_embed:
        #     inputs = inputs * math.sqrt(self._units)

        # Positional Encoding
        # steps = F.contrib.arange_like(inputs, axis=1)
        # positional_embed = F.Embedding(steps, position_weight, self._max_length, self._units)
        # inputs = F.broadcast_add(inputs, F.expand_dims(positional_embed, axis=0))

        # if self._dropout:
        #     inputs = self.dropout_layer(inputs)

        # if self._norm_inputs:
        #     inputs = self.layer_norm(inputs)

        additional_outputs = []
        attention_weights_l = []
        outputs = inputs
        for cell in self.transformer_cells:
            outputs, attention_weights = cell(outputs, mem_value, mask, mem_mask)
            if self._output_attention:
                attention_weights_l.append(attention_weights)
        if self._output_attention:
            additional_outputs.extend(attention_weights_l)

        if valid_length is not None:
            outputs = F.SequenceMask(outputs, sequence_length=valid_length,
                                     use_sequence_length=True, axis=1)
        return outputs, states, additional_outputs


class TransformerDecoder(_BaseTransformerDecoder, Seq2SeqDecoder):
    """Transformer Decoder.

    Multi-step ahead decoder for use during training with teacher forcing.

    Parameters
    ----------
    attention_cell : AttentionCell or str, default 'multi_head'
        Arguments of the attention cell.
        Can be 'multi_head', 'scaled_luong', 'scaled_dot', 'dot', 'cosine', 'normed_mlp', 'mlp'
    num_layers : int
        Number of attention layers.
    units : int
        Number of units for the output.
    hidden_size : int
        number of units in the hidden layer of position-wise feed-forward networks
    max_length : int
        Maximum length of the input sequence. This is used for constructing position encoding
    num_heads : int
        Number of heads in multi-head attention
    scaled : bool
        Whether to scale the softmax input by the sqrt of the input dimension
        in multi-head attention
    scale_embed : bool, default True
        Whether to scale the input embeddings by the sqrt of the `units`.
    norm_inputs : bool, default True
        Whether to normalize the input embeddings with LayerNorm. If dropout is
        enabled, normalization happens after dropout is applied to inputs.
    dropout : float
        Dropout probability.
    use_residual : bool
        Whether to use residual connection.
    output_attention: bool
        Whether to output the attention weights
    weight_initializer : str or Initializer
        Initializer for the input weights matrix, used for the linear
        transformation of the inputs.
    bias_initializer : str or Initializer
        Initializer for the bias vector.
    prefix : str, default 'rnn_'
        Prefix for name of `Block`s
        (and name of weight if params is `None`).
    params : Parameter or None
        Container for weight sharing between cells.
        Created if `None`.
    """