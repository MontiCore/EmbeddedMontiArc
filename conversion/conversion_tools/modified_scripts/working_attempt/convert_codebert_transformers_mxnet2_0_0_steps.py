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
# Library Versions: gluonnlp 1.0.0.dev20210116, mxnet 2.0.0b20210116, torch 1.4.0, transformers 4.2.1

import os
import argparse
import pprint as pp

import mxnet as mx
import numpy as np
from numpy.testing import assert_allclose

import torch
import transformers
import gluonnlp as nlp

mx.npx.set_np()

def parse_args():
    parser = argparse.ArgumentParser(description='Convert the huggingface CodeBERT Model to Gluon.')
    parser.add_argument('--save_dir', type=str, default=None,
                        help='Directory path to save the converted model.')
    parser.add_argument('--gpu', type=int, default=None,
                        help='The single gpu to run mxnet, (e.g. --gpu 0) the default device is cpu.')
    return parser.parse_args()

def convert_params(hf_model, hf_tokenizer, gluon_model, ctx):
    print('converting params')
    # output all hidden states for testing
    gluon_model._output_all_encodings = True
    gluon_model.encoder._output_all_encodings = True

    gluon_model.initialize(ctx=ctx)
    gluon_model.hybridize()
    gluon_params = gluon_model.collect_params()
    num_layers = gluon_model.num_layers
    hf_params = hf_model.state_dict()

    for layer_id in range(num_layers):
        hf_atten_prefix = 'encoder.layer.{}.attention.self.'.format(layer_id)

        hf_q_weight = hf_params[hf_atten_prefix + 'query.weight'].cpu().numpy()
        hf_k_weight = hf_params[hf_atten_prefix + 'key.weight'].cpu().numpy()
        hf_v_weight = hf_params[hf_atten_prefix + 'value.weight'].cpu().numpy()
        hf_q_bias = hf_params[hf_atten_prefix + 'query.bias'].cpu().numpy()
        hf_k_bias = hf_params[hf_atten_prefix + 'key.bias'].cpu().numpy()
        hf_v_bias = hf_params[hf_atten_prefix + 'value.bias'].cpu().numpy()

        gl_qkv_prefix = 'encoder.all_layers.{}.attn_qkv.'.format(layer_id)
        gl_qkv_weight = gluon_params[gl_qkv_prefix + 'weight']
        gl_qkv_bias = gluon_params[gl_qkv_prefix + 'bias']
        gl_qkv_weight.set_data(
            np.concatenate([hf_q_weight, hf_k_weight, hf_v_weight], axis=0))
        gl_qkv_bias.set_data(
            np.concatenate([hf_q_bias, hf_k_bias, hf_v_bias], axis=0))

        for k, v in [
            ('attention.output.dense.weight', 'attention_proj.weight'),
            ('attention.output.dense.bias', 'attention_proj.bias'),
            ('attention.output.LayerNorm.weight', 'layer_norm.gamma'),
            ('attention.output.LayerNorm.bias', 'layer_norm.beta'),
            ('intermediate.dense.weight', 'ffn.ffn_1.weight'),
            ('intermediate.dense.bias', 'ffn.ffn_1.bias'),
            ('output.dense.weight', 'ffn.ffn_2.weight'),
            ('output.dense.bias', 'ffn.ffn_2.bias'),
            ('output.LayerNorm.weight', 'ffn.layer_norm.gamma'),
            ('output.LayerNorm.bias', 'ffn.layer_norm.beta')
        ]:
            # TODO upper part has same prefix, use variable?
            hf_name = 'encoder.layer.{}.{}'.format(layer_id, k)
            gl_name = 'encoder.all_layers.{}.{}'.format(layer_id, v)
            gluon_params[gl_name].set_data(hf_params[hf_name].cpu().numpy())

    for k, v in [
        ('word_embeddings.weight', 'word_embed.weight'),
        ('LayerNorm.weight', 'embed_ln.gamma'),
        ('LayerNorm.bias', 'embed_ln.beta'),
    ]:
        hf_embed_name = "embeddings." + k
        gluon_params[v].set_data(hf_params[hf_embed_name].cpu().numpy())

    # position embed weight
    padding_idx = hf_tokenizer.pad_token_id
    hf_pos_embed_name = 'embeddings.position_embeddings.weight'
    gl_pos_embed_name = 'pos_embed._embed.weight'
    hf_wo_pad = hf_params[hf_pos_embed_name].cpu().numpy()[padding_idx + 1:, :]
    gluon_params[gl_pos_embed_name].set_data(hf_wo_pad)

    pp.pprint(list(zip(list(gluon_model.collect_params().keys()), [gluon_model.collect_params()[k].shape for k in gluon_model.collect_params().keys()])))
    pp.pprint(list(zip(list(hf_params.keys()), [hf_params[k].shape for k in hf_params.keys()])))
    print(gluon_model)
    print(hf_model)


def test_model(hf_model, hf_tokenizer, gluon_model, gpu):
    print('testing model')
    ctx = mx.gpu(gpu) if gpu is not None else mx.cpu()
    batch_size = 3
    seq_length = 32
    vocab_size = hf_model.config.vocab_size
    padding_id = hf_tokenizer.pad_token_id
    input_ids = np.random.randint(padding_id + 1, vocab_size, (batch_size, seq_length))
    valid_length = np.random.randint(seq_length // 2, seq_length, (batch_size,))

    for i in range(batch_size):  # add padding, not sure if necessary for hf codebert
        input_ids[i, valid_length[i]:] = padding_id

    gl_input_ids = mx.np.array(input_ids, dtype=np.int32, ctx=ctx)
    gl_valid_length = mx.np.array(valid_length, dtype=np.int32, ctx=ctx)

    hf_input_ids = torch.from_numpy(input_ids).cpu()
    hf_model.eval()

    gl_all_hiddens, gl_pooled = gluon_model(gl_input_ids, gl_valid_length)

    # create attention mask for hf model
    hf_valid_length = np.zeros((batch_size, seq_length))
    for i in range(batch_size):
        hf_valid_length[i][:valid_length[i]] = 1
    hf_valid_length = torch.from_numpy(hf_valid_length)

    hf_outputs = hf_model(hf_input_ids, attention_mask=hf_valid_length, output_hidden_states=True)
    hf_all_hiddens = hf_outputs['hidden_states']

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
                1E-4,
                1E-4
            )

def convert_huggingface_model(args):
    if not args.save_dir:
        args.save_dir = os.path.basename('./codebert_gluon')
    if not os.path.exists(args.save_dir):
        os.makedirs(args.save_dir)

    # load and save huggingface model
    hf_tokenizer = transformers.RobertaTokenizer.from_pretrained("microsoft/codebert-base")
    hf_model = transformers.RobertaModel.from_pretrained("microsoft/codebert-base")

    # we have to zero these otherwise the test will fail, because these don't exist in gluonnlp
    hf_model.embeddings.token_type_embeddings.weight.data = torch.zeros_like(
        hf_model.embeddings.token_type_embeddings.weight
    )
    
    hf_model.save_pretrained(args.save_dir)
    hf_tokenizer.save_pretrained(args.save_dir)

    # start with RoBERTaModel base
    RobertaModel, gluon_cfg, _, local_params_path, _ = nlp.models.get_backbone('fairseq_roberta_base')
    gluon_model = RobertaModel.from_cfg(gluon_cfg)
    gluon_model.load_parameters(local_params_path)

    ctx = mx.gpu(args.gpu) if args.gpu is not None else mx.cpu()
    convert_params(hf_model, hf_tokenizer, gluon_model, ctx)
    test_model(hf_model, hf_tokenizer, gluon_model, args.gpu)

    gluon_model.export(os.path.join(args.save_dir, 'codebert'))
    print('Exported the CodeBERT model to {}'.format(os.path.join(args.save_dir)))
    print('Conversion finished!')

if __name__ == '__main__':
    args = parse_args()
    convert_huggingface_model(args)
