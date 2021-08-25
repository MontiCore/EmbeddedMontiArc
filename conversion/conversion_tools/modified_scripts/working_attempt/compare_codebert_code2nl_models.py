import codebert_models as mxmod
import codebert_code2nl.model as ptmod
import codebert_code2nl.run as ptrun
import convert_code2nl_data_mxnet1_7_0 as conv
import codebert_code2nl_mxnet1_7_0 as mxrun
import codebert_hyper_params as hyp

from transformers import (AdamW, get_linear_schedule_with_warmup,
                          RobertaConfig, RobertaModel, RobertaTokenizer)
                          
import argparse
import torch
import random
import numpy as np
import mxnet as mx
import gluonnlp as nlp
from torch.utils.data import DataLoader, SequentialSampler, TensorDataset
from itertools import cycle
from collections import namedtuple

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", default='/home/makua/Documents/Datasets/CodeSearchNet/java',
        help="The folder where some training data in the format for the pytorch model is")
    parser.add_argument("--symbol_file", default='./codebert_gluon/model/codebert-symbol.json', type=str,
        help="Symbol file from the pretrained mxnet model output by the conversion script")
    parser.add_argument("--weight_file", default='./codebert_gluon/model/codebert-0000.params', type=str,
        help="Weight file from the pretrained mxnet model output by the conversion script")
    parser.add_argument("--embed_symbol_file", default='./codebert_gluon/model/codebert_embedding-symbol.json', type=str,
        help="Symbol file from the pretrained mxnet embed output by the conversion script")
    parser.add_argument("--embed_weight_file", default='./codebert_gluon/model/codebert_embedding-0000.params', type=str,
        help="Weight file from the pretrained mxnet embed output by the conversion script")
    return parser.parse_args()

def to_pytorch_tensor_ds(features):
    all_source_ids = torch.tensor([f.source_ids for f in features], dtype=torch.long)
    all_source_mask = torch.tensor([f.source_mask for f in features], dtype=torch.long)
    all_target_ids = torch.tensor([f.target_ids for f in features], dtype=torch.long)
    all_target_mask = torch.tensor([f.target_mask for f in features], dtype=torch.long)   
    return TensorDataset(all_source_ids, all_source_mask, all_target_ids, all_target_mask)   

# dev, train.jsonl and test, test.jsonl
def get_pytorch_dataloader(data_dir, stage, filename, limit):
    param_dict = hyp.get_training_hparams(True)
    hparams = namedtuple("HParams", param_dict.keys())(*param_dict.values())
    tokenizer = RobertaTokenizer.from_pretrained("microsoft/codebert-base")
    examples = ptrun.read_examples("{}/{}".format(data_dir, filename))[:limit]
    features = ptrun.convert_examples_to_features(examples, tokenizer, hparams, stage=stage)
    dataset = to_pytorch_tensor_ds(features)
    sampler = SequentialSampler(dataset)
    return DataLoader(dataset, sampler=sampler, batch_size=param_dict['batch_size'])

def get_mxnet_data(data_dir):
    param_dict = hyp.get_training_hparams(True)
    tokenizer = RobertaTokenizer.from_pretrained('microsoft/codebert-base')
    stages = ['train', 'test']
    train_data = conv.get_stage_data('train', tokenizer, data_dir, True)
    test_data = conv.get_stage_data('test', tokenizer, data_dir, True)
    train_iter = conv.get_data_iterator(
        ['source_ids', 'source_masks'], ['target_ids', 'target_masks'],
        False, param_dict['batch_size'], train_data)
    test_iter = conv.get_data_iterator(
        ['source_ids', 'source_masks'], ['target_ids', 'target_masks'],
        False, param_dict['batch_size'], test_data)
    return train_iter, test_iter

def get_pt_seq2seq():
    param_dict = hyp.get_training_hparams(True)
    config_class, model_class, tokenizer_class = RobertaConfig, RobertaModel, RobertaTokenizer
    config = config_class.from_pretrained('microsoft/codebert-base')
    tokenizer = tokenizer_class.from_pretrained('microsoft/codebert-base')
    encoder = model_class.from_pretrained('microsoft/codebert-base', config=config)    
    decoder_layer = torch.nn.TransformerDecoderLayer(d_model=config.hidden_size, nhead=config.num_attention_heads)
    decoder = torch.nn.TransformerDecoder(decoder_layer, num_layers=6)
    model = ptmod.Seq2Seq(encoder=encoder, decoder=decoder, config=config,
                  beam_size=param_dict['beam_size'], max_length=param_dict['max_target_length'],
                  sos_id=tokenizer.cls_token_id, eos_id=tokenizer.sep_token_id, compare_mode=True)
    return model

def train_pt_model(pt_seq2seq, pt_train, weight_decay):
    param_dict = hyp.get_training_hparams(True)
    no_decay = ['bias', 'LayerNorm.weight']
    device = torch.device('cpu')
    # the model doesnt use weight decay but we will leave this for now
    optimizer_grouped_parameters = [
        {'params': [
            p for n, p in pt_seq2seq.named_parameters() 
            if not any(nd in n for nd in no_decay)
        ], 'weight_decay': weight_decay},
        {'params': [
            p for n, p in pt_seq2seq.named_parameters() 
            if any(nd in n for nd in no_decay)
        ], 'weight_decay': 0.0}
    ]
    optimizer = AdamW(
        optimizer_grouped_parameters, 
        lr=param_dict['learning_rate'], 
        eps=param_dict['adam_epsilon']
    )
    scheduler = get_linear_schedule_with_warmup(
        optimizer, num_warmup_steps=0,
        num_training_steps=param_dict['train_steps']
    )
    pt_seq2seq.train()
    pt_train_iter = cycle(pt_train)
    steps_done, cumul_loss = 0, 0
    for step in range(param_dict['train_steps']):
        batch = next(pt_train_iter)
        batch = tuple(t.to(device) for t in batch)
        source_ids, source_mask, target_ids, target_mask = batch
        lm_logits,loss, _, _ = pt_seq2seq(
            source_ids = source_ids,
            source_mask = source_mask,
            target_ids = target_ids,
            target_mask = target_mask
        )
        cumul_loss += loss.item()
        train_loss = round(cumul_loss/(steps_done+1), 4)

        steps_done += 1
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
        scheduler.step()
    return pt_seq2seq

def compare_preds(pt_preds, mx_preds):
    return None

def tandem_test_models(pt_seq2seq, mx_seq2seq, pt_test, mx_test, mx_ctx):
    pt_seq2seq.eval()
    pt_test_iter = cycle(pt_test)
    num_batches = mx_test.num_data // mx_test.batch_size
    pt_preds = []
    pt_probs = []
    mx_probs = []
    mx_preds = []
    for _ in range(num_batches):
        mx_batch = mx_test.next()
        pt_batch = next(pt_test_iter)
        pt_sids, pt_smasks, _, _ = pt_batch
        mx_sids, mx_smasks, _, _ = mxrun.get_seqs_from_batch(mx_batch, mx_ctx)
        for s_id, s_msk in zip(mx_sids, mx_smasks):
            mx_pred, mx_prob = mx_seq2seq(s_id, s_msk)
            mx_preds.append(mx_pred)
            mx_probs.append(mx_prob)
        

        pt_pred, pt_prob = pt_seq2seq(pt_sids, pt_smasks)
        pt_preds.append(pt_pred)
        pt_probs.append(pt_prob)
        print(pt_prob[0][0])
        print(mx_prob[0][0])

def set_seed():
    seed = 42
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    mx.random.seed(seed)

if __name__ == '__main__':
    args = parse_args()
    set_seed()
    mx_ctx = [mx.cpu()]
    param_dict = hyp.get_training_hparams(True)
    pt_train = get_pytorch_dataloader(
        args.data_dir, 'dev', 'train.jsonl', param_dict['limit_train_samples'])
    pt_test = get_pytorch_dataloader(
        args.data_dir, 'test', 'test.jsonl', param_dict['limit_test_samples'])
    mx_train, mx_test = get_mxnet_data(args.data_dir)
    mx_seq2seq = mxrun.get_seq2seq(
        args.symbol_file, args.weight_file, 
        args.embed_symbol_file, args.embed_weight_file, 
        mx_ctx, True, True
    )
    pt_seq2seq = get_pt_seq2seq()
    pt_seq2seq = train_pt_model(pt_seq2seq, pt_train, 0.0)
    mx_seq2seq = mxrun.train_model(mx_seq2seq, mx_train, mx_ctx, True)
    # print(all_lm_logits_pt[0])
    # print(all_lm_logits_pt[0].shape)
    # print(all_lm_logits_mx[0])
    #tandem_test_models(pt_seq2seq, mx_seq2seq, pt_test, mx_test, mx_ctx)