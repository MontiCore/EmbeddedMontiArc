# RoBERTa Equivalencies
# found by looking at conversion/conversion_tools/convert_roberta_original_pytorch_checkpoint_to_pytorch.py
# Huggingface -> Fairseq -> gluonnlp
#
# MODEL CONFIG
#
# RobertaConfig.vocab_size -> FairseqRobertaModel.model.decoder.sentence_encoder.embed_tokens.num_embeddings,
# RobertaConfig.hidden_size -> FairseqRobertaModel.args.encoder_embed_dim,
# RobertaConfig.num_hidden_layers -> FairseqRobertaModel.args.encoder_layers,
# RobertaConfig.num_attention_heads -> FairseqRobertaModel.args.encoder_attention_heads
# RobertaConfig.intermediate_size -> FairseqRobertaModel.args.encoder_ffn_embed_dim
# RobertaConfig.max_position_embeddings -> 512 (514 - 2 for the padding and some other symbol at the beginning)
# RobertaConfig.type_vocab_size -> 1
# RobertaConfig.layer_norm_eps -> 1e-5
#
# WEIGHTS/PARAMS
#
# RobertaForMaskedLM.roberta.embeddings.word_embeddings.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.embed_tokens.weight
# RobertaForMaskedLM.roberta.embeddings.position_embeddings.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.embed_positions.weight
# RobertaForMaskedLM.roberta.embeddings.token_type_embeddings.weight.data -> torch.zeros_like(model.roberta.embeddings.token_type_embeddings.weight)  # just zero them out b/c RoBERTa doesn't use them.
# RobertaForMaskedLM.roberta.embeddings.LayerNorm.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.emb_layer_norm.weight
# RobertaForMaskedLM.roberta.embeddings.LayerNorm.bias -> FairseqRobertaModel.model.decoder.sentence_encoder.emb_layer_norm.bias
#
# FOR EVERY HIDDEN LAYER
#
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.self.query.weight.data -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.q_proj.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.self.query.bias.data -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.q_proj.bias
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.self.key.weight.data -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.k_proj.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.self.key.bias.data -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.k_proj.bias
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.self.value.weight.data -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.v_proj.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.self.value.bias.data -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.v_proj.bias
#
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.output.dense.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.out_proj.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.output.dense.bias -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn.out_proj.bias
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.output.LayerNorm.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn_layer_norm.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].attention.output.LayerNorm.bias -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].self_attn_layer_norm.bias
#
# RobertaForMaskedLM.roberta.encoder.layer[i].intermediate.dense.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].fc1.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].intermediate.dense.bias -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].fc1.bias
#
# RobertaForMaskedLM.roberta.encoder.layer[i].output.dense.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].fc2.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].output.dense.bias -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].fc2.bias
# RobertaForMaskedLM.roberta.encoder.layer[i].output.LayerNorm.weight -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].final_layer_norm.weight
# RobertaForMaskedLM.roberta.encoder.layer[i].output.LayerNorm.bias -> FairseqRobertaModel.model.decoder.sentence_encoder.layers[i].final_layer_norm.bias
#
# The script below is an adapted version of the RoBERTA fairseq to gluonnlp script found in their github
# In addition to the parameter mapping I had to remove mlm parts, zero the token type embeddings weights, and adjust the padding part
# Library Versions: gluonnlp Version: 0.10.0, mxnet 1.7.0.post2, torch 1.4.0, transformers 4.2.1

# Use BERTEncoder class in gluonnlp 0.10.0 and look at the _load_pretrained_params function at https://nlp.gluon.ai/_modules/gluonnlp/model/bert.html#roberta_12_768_12
# next steps, look at distilbert and encoder._collect_params_with_prefix() and compare to huggingface

import os
import re
import sys
import json
import shutil
import argparse
import pprint as pp

import mxnet as mx
from mxnet.gluon.block import HybridBlock
import numpy as np
from numpy.testing import assert_allclose

import torch
import transformers
from gluonnlp.model import BERTEncoder, BERTModel

class RoBERTaModelWPoolerTest(BERTModel):
# analogous to gluonnlp 0.10.0 nlp.model.RobertaModel but it allows use_pooler and use_token_type_embed
    def __init__(self, 
        encoder, 
        vocab_size=None,
        token_type_vocab_size=None,
        units=None,
        embed_size=None, 
        embed_initializer=None,
        word_embed=None,
        token_type_embed=None,
        use_pooler=None,
        use_decoder=None,
        use_classifier=None,
        use_token_type_embed=None, 
        prefix=None, 
        params=None
    ):
        super(RoBERTaModelWPoolerTest, self).__init__(
            encoder, 
            vocab_size=vocab_size,
            token_type_vocab_size=token_type_vocab_size, 
            units=units,
            embed_size=embed_size,
            embed_initializer=embed_initializer,
            word_embed=word_embed,
            token_type_embed=token_type_embed,
            use_pooler=use_pooler,
            use_decoder=use_decoder,
            use_classifier=use_classifier, 
            use_token_type_embed=use_token_type_embed,
            prefix=prefix, 
            params=params
        )

    def __call__(self, inputs, token_types=None, valid_length=None, masked_positions=None):
        return super(RoBERTaModelWPoolerTest, self).__call__(
            inputs, token_types=token_types, valid_length=valid_length,
            masked_positions=masked_positions
        )

class RoBERTaModelWPooler(RoBERTaModelWPoolerTest):
    def hybrid_forward(self, F, inputs, token_types, valid_length=None, masked_positions=None):
        # remove single dim entries from the valid_length input, needed for compatibility with EMADL LoadNetwork layer
        valid_length = mx.symbol.squeeze(valid_length)
        outputs = super(RoBERTaModelWPooler, self).hybrid_forward(
            F, inputs, token_types, valid_length=valid_length, masked_positions=masked_positions
        )
        # only return the last output (pooler output) to make compatible with EMADL LoadNetwork layer
        return outputs[-1]

def parse_args():
    parser = argparse.ArgumentParser(description='Convert the huggingface CodeBERT Model to Gluon.')
    parser.add_argument('--save_dir', type=str, default=None,
        help='Directory path to save the converted model.')
    parser.add_argument('--test', action='store_true',
        help='If the model should be tested for equivalence after conversion, no model is output')
    return parser.parse_args()

def get_gluon_model_arch(hf_cfg, ctx, args):
    enc_hyper_params = {
        'num_layers': 12,
        'units': 768,
        'hidden_size': 3072,
        'max_length': 512,
        'num_heads': 12,
        'dropout': 0.1,
        'output_attention': False,
        'output_all_encodings': True if args.test else False,
        'weight_initializer': None,
        'bias_initializer': 'zeros',
        'prefix': None,
        'params': None,
        'activation': 'gelu',
        'layer_norm_eps': 1e-5
    }

    hyper_params = {
        'vocab_size': hf_cfg.vocab_size,
        'token_type_vocab_size': 1,
        'units': 768,
        'embed_size': 768,
        'embed_initializer': None,
        'word_embed': None,
        'token_type_embed': None,
        'use_pooler': True,
        'use_decoder': False,
        'use_classifier': False,
        'use_token_type_embed': True,
        'prefix': "robertamodelwpooler0_",
        'params': None
    }

    gluon_encoder = BERTEncoder(
        num_layers=enc_hyper_params['num_layers'],
        units=enc_hyper_params['units'],
        hidden_size=enc_hyper_params['hidden_size'],
        max_length=enc_hyper_params['max_length'],
        num_heads=enc_hyper_params['num_heads'],
        dropout=enc_hyper_params['dropout'],
        output_attention=enc_hyper_params['output_attention'],
        output_all_encodings=enc_hyper_params['output_all_encodings'],
        weight_initializer=enc_hyper_params['weight_initializer'],
        bias_initializer=enc_hyper_params['bias_initializer'],
        prefix=enc_hyper_params['prefix'],
        params=enc_hyper_params['params'],
        activation=enc_hyper_params['activation'],
        layer_norm_eps=enc_hyper_params['layer_norm_eps']
    )

    if args.test:
        gluon_model = RoBERTaModelWPoolerTest(
            encoder=gluon_encoder, 
            vocab_size=hyper_params['vocab_size'],
            token_type_vocab_size=hyper_params['token_type_vocab_size'],
            units=hyper_params['units'],
            embed_size=hyper_params['embed_size'],
            embed_initializer=hyper_params['embed_initializer'],
            word_embed=hyper_params['word_embed'],
            token_type_embed=hyper_params['token_type_embed'],
            use_pooler=hyper_params['use_pooler'],
            use_decoder=hyper_params['use_decoder'],
            use_classifier=hyper_params['use_classifier'],
            use_token_type_embed=hyper_params['use_token_type_embed'],
            prefix=hyper_params['prefix'],
            params=hyper_params['params']
        )
    else:
        gluon_model = RoBERTaModelWPooler(
            encoder=gluon_encoder, 
            vocab_size=hyper_params['vocab_size'],
            token_type_vocab_size=hyper_params['token_type_vocab_size'],
            units=hyper_params['units'],
            embed_size=hyper_params['embed_size'],
            embed_initializer=hyper_params['embed_initializer'],
            word_embed=hyper_params['word_embed'],
            token_type_embed=hyper_params['token_type_embed'],
            use_pooler=hyper_params['use_pooler'],
            use_decoder=hyper_params['use_decoder'],
            use_classifier=hyper_params['use_classifier'],
            use_token_type_embed=hyper_params['use_token_type_embed'],
            prefix=hyper_params['prefix'],
            params=hyper_params['params']
        )

    gluon_model.initialize(ctx=ctx) # unsure if it should be init with normal
    gluon_model.hybridize()
    return gluon_model

def convert_params(hf_model, hf_tokenizer, hf_cfg, args):
    print('Converting Parameters...')
    # use nlp.model.get_model('roberta_12_768_12', dataset_name='openwebtext_ccnews_stories_books_cased', use_decoder=False) and look at its
    # source to get an idea of how to initialize a blank model you can use
    #
    # the function used is here
    # https://github.com/dmlc/gluon-nlp/blob/14559518a75081469bfba14150ded2dc97c13902/src/gluonnlp/model/bert.py#L1459
    #
    ctx = mx.cpu()
    gluon_model = get_gluon_model_arch(hf_cfg, ctx, args)
    gluon_params = gluon_model.collect_params()
    hf_params = hf_model.state_dict()

    num_layers = hf_cfg.num_hidden_layers
    for layer_id in range(num_layers):
        hf_prefix = 'encoder.layer.{}.'.format(layer_id)
        hf_atten_prefix = hf_prefix + 'attention.self.'
        gl_prefix = 'bertencoder0_transformer{}'.format(layer_id)
        gl_qkv_prefix = gl_prefix + '_dotproductselfattentioncell0_'

        for name in [
            'query{}weight', 'key{}weight', 'value{}weight', 
            'query{}bias', 'key{}bias', 'value{}bias'
        ]:
            gl_name = gl_qkv_prefix + name.format('_')
            hf_name = hf_atten_prefix + name.format('.') 
            gluon_params[gl_name].set_data(arr_to_gl(hf_params[hf_name]))

        for hf_suffix, gl_suffix in [
            ('attention.output.dense.weight', '_proj_weight'),
            ('attention.output.dense.bias', '_proj_bias'),
            ('attention.output.LayerNorm.weight', '_layernorm0_gamma'),
            ('attention.output.LayerNorm.bias', '_layernorm0_beta'),
            ('intermediate.dense.weight', '_positionwiseffn0_ffn_1_weight'),
            ('intermediate.dense.bias', '_positionwiseffn0_ffn_1_bias'),
            ('output.dense.weight', '_positionwiseffn0_ffn_2_weight'),
            ('output.dense.bias', '_positionwiseffn0_ffn_2_bias'),
            ('output.LayerNorm.weight', '_positionwiseffn0_layernorm0_gamma'),
            ('output.LayerNorm.bias', '_positionwiseffn0_layernorm0_beta')
        ]:
            hf_name = hf_prefix + hf_suffix
            gl_name = gl_prefix + gl_suffix
            gluon_params[gl_name].set_data(arr_to_gl(hf_params[hf_name]))



    for hf_name, gl_name in [
        ('embeddings.word_embeddings.weight', 'robertamodelwpooler0_word_embed_embedding0_weight'),
        ('embeddings.token_type_embeddings.weight', 'robertamodelwpooler0_token_type_embed_embedding0_weight'),
        ('embeddings.LayerNorm.weight', 'bertencoder0_layernorm0_gamma'),
        ('embeddings.LayerNorm.bias', 'bertencoder0_layernorm0_beta'),
        ('pooler.dense.weight', 'robertamodelwpooler0_pooler_weight'),
        ('pooler.dense.bias', 'robertamodelwpooler0_pooler_bias')
    ]:
        gluon_params[gl_name].set_data(arr_to_gl(hf_params[hf_name]))

    # position embed weight
    padding_idx = hf_tokenizer.pad_token_id
    hf_pos_embed_name = 'embeddings.position_embeddings.weight'
    gl_pos_embed_name = 'bertencoder0_position_weight'
    hf_wo_pad = arr_to_gl(hf_params[hf_pos_embed_name])[padding_idx + 1:, :]
    gluon_params[gl_pos_embed_name].set_data(hf_wo_pad)
    
    print(gluon_model.collect_params())
    pp.pprint(list(zip(list(hf_params.keys()), [hf_params[k].shape for k in hf_params.keys()])))
    return gluon_model

def arr_to_gl(arr):
    return mx.nd.array(arr.cpu().numpy())

def test_model(hf_model, hf_tokenizer, gluon_model, args):
    print('Performing a short model test...')
    ctx = mx.cpu()
    batch_size = 3
    seq_length = 32
    vocab_size = hf_model.config.vocab_size
    padding_id = hf_tokenizer.pad_token_id
    input_ids = np.random.randint(padding_id + 1, vocab_size, (batch_size, seq_length))
    valid_length = np.random.randint(seq_length // 2, seq_length, (batch_size,))

    # add padding, not sure if necessary for hf codebert
    for i in range(batch_size):  
        input_ids[i, valid_length[i]:] = padding_id

    gl_input_ids = mx.nd.array(input_ids)
    gl_valid_length = mx.nd.array(valid_length)
    gl_token_types = mx.nd.zeros((batch_size, seq_length))

    hf_input_ids = torch.from_numpy(input_ids).cpu()
    hf_model.eval()

    #gl_all_hiddens, gl_pooled
    gl_outs = gluon_model(
        gl_input_ids, 
        token_types=gl_token_types,
        # reshape the inputs from (n,) to (n,1) to mock LoadNetwork layer inputs in EMADL
        valid_length=gl_valid_length.reshape(gl_valid_length.shape[0], 1)
    )

    if args.test:
        print('Performing a long model test...')
        gl_all_hiddens, gl_pooled = gl_outs

        # create attention mask for hf model
        hf_valid_length = np.zeros((batch_size, seq_length))
        for i in range(batch_size):
            hf_valid_length[i][:valid_length[i]] = 1
        hf_valid_length = torch.from_numpy(hf_valid_length)

        hf_outputs = hf_model(hf_input_ids, attention_mask=hf_valid_length, output_hidden_states=True)
        # (num_layers + 1, batch_size, seq_length, hidden_size)
        hf_all_hiddens = hf_outputs['hidden_states'][1:]
        hf_pooled = hf_outputs['pooler_output']

        # check pooling output
        assert_allclose(gl_pooled.asnumpy(), hf_pooled.detach().cpu().numpy(), 1E-3, 1E-3)

        # checking all_encodings_outputs
        num_layers = hf_model.config.num_hidden_layers
        for i in range(num_layers + 1):
            gl_hidden = gl_all_hiddens[i].asnumpy()
            hf_hidden = hf_all_hiddens[i]
            hf_hidden = hf_hidden.detach().cpu().numpy()
            for j in range(batch_size):
                assert_allclose(
                    gl_hidden[j, :valid_length[j], :],
                    hf_hidden[j, :valid_length[j], :],
                    1E-3,
                    1E-3
                )


def export_model(save_dir, gluon_model):
    gluon_model.encoder.output_all_encodings = False
    gluon_model.export(os.path.join(save_dir, 'codebert'))
    print('Exported the CodeBERT model to {}'.format(os.path.join(save_dir)))


def convert_huggingface_model(args):
    if not args.save_dir:
        args.save_dir = os.path.basename('./codebert_gluon')
    if not os.path.exists(args.save_dir):
        os.makedirs(args.save_dir)

    # load and save huggingface model
    hf_tokenizer = transformers.RobertaTokenizer.from_pretrained("microsoft/codebert-base")
    hf_model = transformers.RobertaModel.from_pretrained("microsoft/codebert-base")

    gluon_model = convert_params(hf_model, hf_tokenizer, hf_model.config, args)
    
    # test currently not passing
    test_model(hf_model, hf_tokenizer, gluon_model, args)

    print('Conversion finished!')
    if not args.test:
        export_model(args.save_dir, gluon_model)
    else:
        print('Testing finished!')

if __name__ == '__main__':
    args = parse_args()
    convert_huggingface_model(args)