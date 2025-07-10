# extracted and adapted from the original run.py script for codebert code2nl
# to work with mxnet/gluon/gluonnlp

import json
from transformers.models.roberta import RobertaTokenizer
import numpy as np
import h5py
import argparse
import os
import codebert_hyper_params as hp
import mxnet as mx

class Example(object):
    """A single training/test example."""
    def __init__(self,
                 idx,
                 source,
                 target,
                 ):
        self.idx = idx
        self.source = source
        self.target = target

class InputFeatures(object):
    """A single training/test features for a example."""
    def __init__(
        self,
        example_id,
        source_ids,
        target_ids,
        source_mask,
        target_mask,
    ):
        self.example_id = example_id
        self.source_ids = source_ids
        self.target_ids = target_ids
        self.source_mask = source_mask
        self.target_mask = target_mask

def read_examples(filename, limit):
    print('Reading {} data...'.format(filename))
    """Read examples from filename."""
    examples=[]
    with open(filename,encoding="utf-8") as f:
        for idx, line in enumerate(f):
            if limit > 0 and idx >= limit:
                break
            line=line.strip()
            js=json.loads(line)
            if 'idx' not in js:
                js['idx']=idx
            code=' '.join(js['code_tokens']).replace('\n',' ')
            code=' '.join(code.strip().split())
            nl=' '.join(js['docstring_tokens']).replace('\n','')
            nl=' '.join(nl.strip().split())            
            examples.append(
                Example(
                    idx=idx,
                    source=code,
                    target =nl,
                ) 
            )
    return examples

# TODO reuse the code block in here to create a new func
def convert_examples_to_features(examples, tokenizer, max_source_length, max_target_length, stage=None):
    print('Converting {} examples to features...'.format(str(len(examples))))
    features = []
    for example_index, example in enumerate(examples):
        #source
        source_tokens = tokenizer.tokenize(example.source)[:max_source_length-2]
        source_tokens =[tokenizer.cls_token]+source_tokens+[tokenizer.sep_token]
        source_ids =  tokenizer.convert_tokens_to_ids(source_tokens) 
        source_mask = len(source_tokens)
        padding_length = max_source_length - len(source_ids)
        source_ids+=[tokenizer.pad_token_id]*padding_length
        #target
        target_tokens = tokenizer.tokenize(example.target)[:max_target_length-2]
        target_tokens = [tokenizer.cls_token]+target_tokens+[tokenizer.sep_token]      
        target_ids = tokenizer.convert_tokens_to_ids(target_tokens)
        target_mask = len(target_tokens)
        padding_length = max_target_length - len(target_ids)
        target_ids+=[tokenizer.pad_token_id]*padding_length
       
        features.append(
            InputFeatures(
                 example_index,
                 source_ids,
                 target_ids,
                 source_mask,
                 target_mask,
            )
        )
    return features

def get_stage_data(stage, tokenizer, data_dir, test_run):
    datafile = '{}/{}.jsonl'.format(data_dir, stage)
    data_params = hp.get_training_hparams(test_run)
    stage_limit = data_params['limit_{}_samples'.format(stage)]
    stage_examples = read_examples(datafile, limit=stage_limit)
    stage_features = convert_examples_to_features(
        stage_examples, tokenizer,
        data_params['max_source_length'],
        data_params['max_target_length'],
        stage=stage
    )
    source_ids = np.array([f.source_ids for f in stage_features], dtype=np.int64)
    source_masks = np.array([f.source_mask for f in stage_features], dtype=np.int64)
    target_ids = np.array([f.target_ids for f in stage_features], dtype=np.int64)
    target_masks = np.array([f.target_mask for f in stage_features], dtype=np.int64)    
    return {
        'source_ids': source_ids, 
        'source_masks': source_masks, 
        'target_ids': target_ids, 
        'target_masks': target_masks
    }

def write_dataset_to_disk(stage, data, savedir):
    datafile = '{}/{}.h5'.format(savedir, stage)
    print('Writing {} data to {}...'.format(stage, datafile))
    if not os.path.exists(savedir):
        os.makedirs(savedir)
    f = h5py.File(datafile, 'w')
    for k, v in data.items():
        f.create_dataset(k, data=v)

def get_data_iterator(inputs, outputs, shuffle, batch_size, loaded_dict):
    # similar to what is done in the CNNArch2Gluon generated data loader
    input_dict = {}
    output_dict = {}
    for k in inputs:
        input_dict[k] = loaded_dict[k]
    for k in outputs:
        output_dict[k] = loaded_dict[k]
    data_iterator = mx.io.NDArrayIter(
        data=input_dict, label=output_dict, shuffle=shuffle, batch_size=batch_size)
    return data_iterator

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test_run', action='store_true', default=False, 
        help='Output the data for a test run of the model')
    parser.add_argument("--save_dir", default='./codebert_gluon/data', 
        help="The path where the processed training data should be saved")
    parser.add_argument("--data_dir", default='/home/makua/Documents/Datasets/CodeSearchNet/java',
        help="The folder where the unprocessed data is, can be found on the codebert github")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    stages = ['train', 'valid', 'test']
    if args.test_run:
        print('Exporting data for test run of model...')
    print('Getting pretrained tokenizer...')
    tokenizer = RobertaTokenizer.from_pretrained('microsoft/codebert-base')
    for stage in stages:
        data = get_stage_data(stage, tokenizer, args.data_dir, args.test_run)
        write_dataset_to_disk(stage, data, args.save_dir)