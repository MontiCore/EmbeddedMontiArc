# original call to script
# cd code2nl

# lang=php #programming language
# lr=5e-5
# batch_size=64
# beam_size=10
# source_length=256
# target_length=128
# data_dir=../data/code2nl/CodeSearchNet
# output_dir=model/$lang
# train_file=$data_dir/$lang/train.jsonl
# dev_file=$data_dir/$lang/valid.jsonl
# eval_steps=1000 #400 for ruby, 600 for javascript, 1000 for others
# train_steps=50000 #20000 for ruby, 30000 for javascript, 50000 for others
# pretrained_model=microsoft/codebert-base #Roberta: roberta-base

# python run.py --do_train --do_eval --model_type roberta --model_name_or_path $pretrained_model --train_filename $train_file --dev_filename $dev_file --output_dir $output_dir --max_source_length $source_length --max_target_length $target_length --beam_size $beam_size --train_batch_size $batch_size --eval_batch_size $batch_size --learning_rate $lr --train_steps $train_steps --eval_steps $eval_steps 

# codebert roberta config

# RobertaConfig {
#   "architectures": [
#     "RobertaModel"
#   ],
#   "attention_probs_dropout_prob": 0.1,
#   "bos_token_id": 0,
#   "eos_token_id": 2,
#   "gradient_checkpointing": false,
#   "hidden_act": "gelu",
#   "hidden_dropout_prob": 0.1,
#   "hidden_size": 768,
#   "initializer_range": 0.02,
#   "intermediate_size": 3072,
#   "layer_norm_eps": 1e-05,
#   "max_position_embeddings": 514,
#   "model_type": "roberta",
#   "num_attention_heads": 12,
#   "num_hidden_layers": 12,
#   "output_past": true,
#   "pad_token_id": 1,
#   "position_embedding_type": "absolute",
#   "transformers_version": "4.2.1",
#   "type_vocab_size": 1,
#   "use_cache": true,
#   "vocab_size": 50265
# }

from convert_codebert_transformers_mxnet1_7_0 import convert_huggingface_model
from gluonnlp.model.transformer import TransformerDecoder
from mxnet.gluon.block import HybridBlock
from mxnet import gluon
import mxnet.gluon.nn as nn
import mxnet as mx
import gluonnlp as nlp
import argparse
import h5py

class Seq2Seq(HybridBlock):
    def __init__(
            self, encoder, decoder, embedding,
            vocab_size=None, hidden_size=None, beam_size=None, 
            max_length=None, sos_id=None, 
            eos_id=None, prefix=None, params=None
        ):
            super().__init__(prefix=prefix, params=params)
            self.encoder = encoder
            self.decoder = decoder
            self.embedding = embedding
            self.bias = mx.ndarray.linalg.extracttrian(mx.nd.ones((2048,2048)))
            self.dense = nn.Dense(hidden_size, in_units=hidden_size, activation='tanh')
            # tie the lm_head and word_embed params together not sure if this is the correct way to do it
            # TODO maybe embedding params object is in dictionary and this is referencing all the params of the embedding layers?
            self.lm_head = self.get_lm_head(vocab_size, hidden_size)
            #self.lsm = mx.npx.log_softmax(axis=-1) # axis = -1 the same as dim = -1? used for prediction part TODO
            self.beam_size=beam_size
            self.max_length=max_length
            self.sos_id=sos_id
            self.eos_id=eos_id
    
    def get_lm_head(self, vocab_size, hidden_size):
        # using params=.. in the Dense constructor doesnt seem to work, so we have to set the weights manually?
        # TODO after setting the weight should we use params= in the lm_head constructor to make them update automatically?
        lm_head = nn.Dense(vocab_size, in_units=hidden_size, use_bias=False)
        lm_head.collect_params()['weight'].set_data(
            self.encoder.collect_params()['robertamodelwpooler0_word_embed_embedding0_weight'].data())
        return lm_head

    def create_target_mask(self, target_ids, valid_length):
        target_mask = mx.nd.zeros_like(target_ids)
        for mask, len in zip(target_mask, valid_length):
            mask[0:len] = 1
        return target_mask

    def forward(self, input_ids=None, input_valid_length=None, target_ids=None, target_valid_length=None):   
        input_token_types = mx.nd.zeros_like(input_ids)
        encoder_output = self.encoder(input_ids, input_token_types, input_valid_length) # we don't permute like in the code2nl model, that okay? TODO
        #encoder_output = outputs[0].permute([1,0,2]).contiguous() not sure how to do
        if target_ids is not None:  
            #attn_mask=-1e4 *(1-self.bias[:target_ids.shape[1],:target_ids.shape[1]]) no option to pass this in gluonnlp TODO
            # could try subclassing the Decoder and change the hybrid_forward function.
            tgt_embeddings = self.embedding(target_ids)
            states = self.decoder.init_state_from_encoder(encoder_output, input_valid_length)
            out = self.decoder(tgt_embeddings, states, valid_length=target_valid_length)
            hidden_states = self.dense(out)
            lm_logits = self.lm_head(hidden_states)
            return lm_logits
        else:
            #Predict 
            preds=[]       
            # zero=torch.cuda.LongTensor(1).fill_(0)     
            # for i in range(source_ids.shape[0]):
            #     context=encoder_output[:,i:i+1]
            #     context_mask=source_mask[i:i+1,:]
            #     beam = Beam(self.beam_size,self.sos_id,self.eos_id)
            #     input_ids=beam.getCurrentState()
            #     context=context.repeat(1, self.beam_size,1)
            #     context_mask=context_mask.repeat(self.beam_size,1)
            #     for _ in range(self.max_length): 
            #         if beam.done():
            #             break
            #         attn_mask=-1e4 *(1-self.bias[:input_ids.shape[1],:input_ids.shape[1]])
            #         tgt_embeddings = self.encoder.embeddings(input_ids).permute([1,0,2]).contiguous()
            #         out = self.decoder(tgt_embeddings,context,tgt_mask=attn_mask,memory_key_padding_mask=(1-context_mask).bool())
            #         out = torch.tanh(self.dense(out))
            #         hidden_states=out.permute([1,0,2]).contiguous()[:,-1,:]
            #         out = self.lsm(self.lm_head(hidden_states)).data
            #         beam.advance(out)
            #         input_ids.data.copy_(input_ids.data.index_select(0, beam.getCurrentOrigin()))
            #         input_ids=torch.cat((input_ids,beam.getCurrentState()),-1)
            #     hyp= beam.getHyp(beam.getFinal())
            #     pred=beam.buildTargetTokens(hyp)[:self.beam_size]
            #     pred=[torch.cat([x.view(-1) for x in p]+[zero]*(self.max_length-len(p))).view(1,-1) for p in pred]
            #     preds.append(torch.cat(pred,0).unsqueeze(0))
                
            # preds=torch.cat(preds,0)                
            # return preds


def get_decoder():
    # put together by looking at the torch decoder
    decoder_hparam = {
        'attention_cell': 'multi_head',
        'num_layers': 6,
        'units': 768,
        'hidden_size': 2048,
        'max_length': 50,
        'num_heads': 12,
        'scaled': True,
        'scale_embed': False,
        'norm_inputs': False,
        # this is slightly different thand torch, because an additional dropout before decoder layers is added
        # the TransformerDecoderLayers also have 3 separate dropout layers and gluon only has 1?
        'dropout': 0.1, 
        'use_residual': True, 
        'output_attention': False, 
        'weight_initializer': None,
        'bias_initializer': 'zeros',
        'prefix': None,
        'params': None
    }
    # gluon TransformerDecoder does a positional encoding before input, does codebert do the same thing?
    # gluonnlp might not do it if position_weight is none?
    # torch model constructs the ffn in the forward call, gluon uses a custom layer for it PositionwiseFFN
    decoder = nlp.model.transformer.TransformerDecoder(
        attention_cell=decoder_hparam['attention_cell'], 
        num_layers=decoder_hparam['num_layers'], 
        units=decoder_hparam['units'], 
        hidden_size=decoder_hparam['hidden_size'],
        max_length=decoder_hparam['max_length'], 
        num_heads=decoder_hparam['num_heads'], 
        scaled=decoder_hparam['scaled'], 
        scale_embed=decoder_hparam['scale_embed'], 
        norm_inputs=decoder_hparam['norm_inputs'],
        dropout=decoder_hparam['dropout'], 
        use_residual=decoder_hparam['use_residual'], 
        output_attention=decoder_hparam['output_attention'], 
        weight_initializer=decoder_hparam['weight_initializer'],
        bias_initializer=decoder_hparam['bias_initializer'], 
        prefix=decoder_hparam['prefix'], 
        params=decoder_hparam['prefix']
    )
    return decoder

def get_seq2seq(encoder, decoder, embedding):
    seq2seq_hparams = {
        'encoder': encoder,
        'decoder': decoder,
        'embedding': embedding,
        'hidden_size': 768,
        'vocab_size': 50265, # TODO correct vocab size? or should it be -2/+2?
        'beam_size': 10,
        'max_target_length': 128,
        'sos_id': 0,
        'eos_id': 2
    }
    seq2seq = Seq2Seq(
        # params taken from cmd line args in codebert repo and roberta config
        seq2seq_hparams['encoder'], 
        seq2seq_hparams['decoder'],
        seq2seq_hparams['embedding'],
        hidden_size=seq2seq_hparams['hidden_size'],
        vocab_size=seq2seq_hparams['vocab_size'],
        beam_size=seq2seq_hparams['beam_size'], 
        max_length=seq2seq_hparams['max_target_length'], 
        sos_id=seq2seq_hparams['sos_id'],
        eos_id=seq2seq_hparams['eos_id'] 
    )
    return seq2seq

def get_data_iterator(inputs, outputs, shuffle, batch_size, filename):
    # similar to what is done in the CNNArch2Gluon generated data loader
    file = h5py.File(filename, 'r')
    input_dict = {}
    output_dict = {}
    for k in inputs:
        input_dict[k] = file[k]
    for k in outputs:
        output_dict[k] = file[k]
    data_iterator = mx.io.NDArrayIter(
        data=input_dict, label=output_dict, shuffle=shuffle, batch_size=batch_size)
    return data_iterator

def load_codebert_encoder_block(symbolFile, weightFile, context):
    inputNames = ['data0', 'data1', 'data2']
    return gluon.nn.SymbolBlock.imports(symbolFile, inputNames, weightFile, ctx=context)

def get_word_embed_subnet(symbol_file, weight_file, ctx):
    # we need access to the word embedding part of the RoBERTaModel, 
    # which cant be directly accessed with mxnets symbol api (through the encoder SymbolBlock)
    # so we have to costruct a new subnet from the symbols up to the layer norm
    # in the pretrained network we pass as the encoder part of the seq2seq model
    sym = mx.sym.load(symbol_file)
    inputs = [mx.sym.var('data0'), mx.sym.var('data1'), mx.sym.var('data2')]
    outputs = sym.get_internals()['bertencoder0_layernorm0_layernorm0_output'] # end of the word embedding part of the model
    block = mx.gluon.SymbolBlock(inputs=inputs, outputs=outputs)
    block.load_parameters(weight_file, ctx=ctx, ignore_extra=True) # load weights into symbolblock
    # TODO do we need to permute the outputs to (1, 0, 2)?
    return block

def get_word_embed_weights(symbol_file, weight_file, ctx):
    # the lm_head layer needs to have the same embedding 'weights' as the embedding layer 
    sym = mx.sym.load(symbol_file)
    inputs = mx.sym.var('data0') #dummy input to return the weight array
    outputs = sym.get_internals()['robertamodelwpooler0_word_embed_embedding0_weight'] # end of the word embedding part of the model
    block = mx.gluon.SymbolBlock(inputs=inputs, outputs=outputs)
    block.load_parameters(weight_file, ctx=ctx, ignore_extra=True) # load weights into symbolblock
    return block(mx.nd.array([])) #dummy input to return the weight array TODO is there a cleaner way to do this?

def train_model(args):
    ctx = [mx.cpu()]
    encoder = load_codebert_encoder_block(args.symbol_file, args.weight_file, ctx)
    decoder = get_decoder()
    embedding = get_word_embed_subnet(args.symbol_file, args.weight_file, ctx)
    seq2seq = get_seq2seq(encoder, decoder, embedding)
    seq2seq.collect_params().initialize(force_reinit=False, ctx=ctx)
    seq2seq.hybridize()
    train_data = get_data_iterator(
        ['source_ids', 'source_masks'], ['target_ids', 'target_masks'],
        True, args.batch_size, args.train_data)
    seq2seq.initialize()
    loss = mx.gluon.loss.SoftmaxCrossEntropyLoss() # TODO parameters? e.g. sparse_label
    # note this is different than the codebert optimizer in two ways
    # 1. differences in calculation, as stated in the BERTAdam optimizer doc
    # 2. it doesn't appear to be able to exclude certain parameters from the optimizer
    # which is done in the codebert code2nl's optimizer. TODO for now.


    # lr is defined in the call to run codebert, epsilon is left as default in the run script, beta1 and 2 are the pytorch default
    optimizer = nlp.optimizer.BERTAdam(learning_rate=5e-5, beta1=0.9, beta2=0.999, epsilon=1e-8)
    trainer = mx.gluon.Trainer(seq2seq.collect_params(), optimizer=optimizer)

    for epoch in range(args.epochs):
        for batch in train_data:
            with mx.autograd.record():
                source_ids = gluon.utils.split_and_load(batch.data[0], ctx_list=ctx, even_split=False)[0]
                source_masks = gluon.utils.split_and_load(batch.data[1], ctx_list=ctx, even_split=False)[0]
                target_ids = gluon.utils.split_and_load(batch.label[0], ctx_list=ctx, even_split=False)[0]
                target_masks = gluon.utils.split_and_load(batch.label[1], ctx_list=ctx, even_split=False)[0]
                lm_logits = seq2seq(source_ids, source_masks, target_ids, target_masks)
                # drop the start of sentence mask tokens? - mb
                active_loss = target_masks[..., 1:].asnumpy().reshape(-1) != 0
                shift_labels = target_ids[..., 1:]
                # Shift so that tokens < n predict n
                shift_logits = lm_logits[..., :-1, :]
                X = shift_logits.reshape(-1, shift_logits.shape(-1))[active_loss]
                y = shift_labels.reshape(-1)[active_loss]
                l = loss(X, y)
                l.backward()
                trainer.step(args.batch_size)
    # target_mask = self.create_target_mask(target_ids, target_valid_length)
    # # Shift so that tokens < n predict n
    # active_loss = target_mask[..., 1:].asnumpy().reshape(-1) != 0
    # #active_loss = target_mask[..., 1:].ne(0).view(-1) == 1
    # shift_logits = lm_logits[..., :-1, :]#.contiguous()
    # shift_labels = target_ids[..., 1:]#.contiguous()
    # # Flatten the tokens
    # # loss_fct = nn.CrossEntropyLoss(ignore_index=-1)
    # # still need to include equivalent to ignore_index? are the losses equivalent?
    # # from_logits flag?
    # loss_fct = mx.gluon.loss.SoftmaxCrossEntropyLoss()
    # loss = loss_fct(shift_logits.reshape(-1, shift_logits.shape(-1))[active_loss],
    #                 shift_labels.reshape(-1)[active_loss])

    # #outputs = loss,loss*active_loss.sum(),active_loss.sum()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', action='store_true', default=True, 
        help='The path where the training data should be saved')
    parser.add_argument("--epochs", default=1, type=int,
        help="The number of epochs for training")
    parser.add_argument('--train_data', default=None, type=str,
        help='The .h5 file where the processed training data is.')
    parser.add_argument("--batch_size", default=8, type=int,
        help="Batch size for training")
    parser.add_argument("--symbol_file", default='./codebert_gluon/codebert-symbol.json', type=str,
        help="Symbol file from the pretrained model output by the conversion script")
    parser.add_argument("--weight_file", default='./codebert_gluon/codebert-0000.params', type=str,
        help="Weight file from the pretrained model output by the conversion script")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    if args.train:
        train_model(args)