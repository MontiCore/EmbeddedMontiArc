import codebert_models as mxmod
import codebert_code2nl.model as ptmod
import codebert_code2nl.run as ptrun
import convert_code2nl_data_mxnet1_7_0 as conv
import codebert_code2nl_mxnet1_7_0 as mxrun
import codebert_hyper_params as hyp

from transformers import (AdamW, get_linear_schedule_with_warmup,
                          RobertaConfig, RobertaModel, RobertaTokenizer)
import argparse
import torch.nn as nn
from itertools import cycle

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

def to_pytorch_tensor_ds(features):
    all_source_ids = torch.tensor([f.source_ids for f in eval_features], dtype=torch.long)
    all_source_mask = torch.tensor([f.source_mask for f in eval_features], dtype=torch.long)
    all_target_ids = torch.tensor([f.target_ids for f in eval_features], dtype=torch.long)
    all_target_mask = torch.tensor([f.target_mask for f in eval_features], dtype=torch.long)   
    return TensorDataset(all_source_ids, all_source_mask, all_target_ids, all_target_mask)   

# dev, train.jsonl and test, test.jsonl
def get_pytorch_dataloader(data_dir, stage, filename):
    param_dict = hyp.get_training_hparams(True)
    hparams = namedtuple("HParams", param_dict.keys())(*param_dict.values())
    tokenizer = RobertaTokenizer.from_pretrained("microsoft/codebert-base")
    examples = ptrun.read_examples("{}/{}".format(data_dir, filename))
    features = ptrun.convert_examples_to_features(examples, tokenizer, hparams, stage=stage)
    dataset = to_pytorch_tensor_ds(features)
    sampler = SequentialSampler(dataset)
    return DataLoader(dataset, sampler=sampler, batch_size=param_dict['batch_size'])

def get_mxnet_data(data_dir):
    tokenizer = RobertaTokenizer.from_pretrained('microsoft/codebert-base')
    stages = ['train', 'test']
    train_data = conv.get_stage_data('train', tokenizer, data_dir, True)
    test_data = conv.get_stage_data('test', tokenizer, data_dir, True)
    num_samples, train_iter = conv.get_data_iterator(
        ['source_ids', 'source_masks'], ['target_ids', 'target_masks'],
        False, batch_size, train_data)
    _, test_iter = conv.get_data_iterator(
        ['source_ids', 'source_masks'], ['target_ids', 'target_masks'],
        False, batch_size, test_data)
    return train_iter, test_iter

def get_pt_seq2seq():
    param_dict = hyp.get_training_hparams(True)
    config_class, model_class, tokenizer_class = RobertaConfig, RobertaModel, RobertaTokenizer
    config = config_class.from_pretrained('microsoft/codebert-base')
    tokenizer = tokenizer_class.from_pretrained('microsoft/codebert-base')
    encoder = model_class.from_pretrained('microsoft/codebert-base', config=config)    
    decoder_layer = nn.TransformerDecoderLayer(d_model=config.hidden_size, nhead=config.num_attention_heads)
    decoder = nn.TransformerDecoder(decoder_layer, num_layers=6)
    model = ptmod.Seq2Seq(encoder=encoder, decoder=decoder, config=config,
                  beam_size=param_dict['beam_size'], max_length=param_dict['max_target_length'],
                  sos_id=tokenizer.cls_token_id, eos_id=tokenizer.sep_token_id)
    return model

def train_pt_model(pt_seq2seq, pt_train, pt_test):
    param_dict = hyp.get_training_hparams(True)
    no_decay = ['bias', 'LayerNorm.weight']
    device = torch.device('cpu')
    optimizer_grouped_parameters = [
        {'params': [
            p for n, p in pt_seq2seq.named_parameters() 
            if not any(nd in n for nd in no_decay)
        ], 'weight_decay': args.weight_decay},
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
    seq2seq.train()
    pt_train_iter = cycle(pt_train)
    steps_done, cumul_loss = 0, 0
    for step in range(param_dict['train_steps']):
        batch = next(train_dataloader)
        batch = tuple(t.to(device) for t in batch)
        source_ids, source_mask, target_ids, target_mask = batch
        loss, _, _ = seq2seq(
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


if __name__ == '__main__':
    args = parse_args()
    pt_train = get_pytorch_dataloader(args.data_dir, 'dev', 'train.jsonl')
    pt_test = get_pytorch_dataloader(args.data_dir, 'test', 'test.jsonl')
    mx_train, mx_test = get_mxnet_data(args.data_dir)
    mx_seq2seq = get_seq2seq(
        args.symbol_file, args.weight_file, 
        args.embed_symbol_file, args.embed_weight_file, 
        [mx.cpu()], True
    )
    pt_seq2seq = get_pt_seq2seq()
    train_pt_model(pt_seq2seq, pt_train, pt_test)
    