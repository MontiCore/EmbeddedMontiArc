import codebert_models as mxmod
import codebert_code2nl.model as ptmod
import convert_code2nl_data_mxnet1_7_0 as conv
import codebert_code2nl.run as ptrun

from transformers.models.roberta import RobertaTokenizer

import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", default='/home/makua/Documents/Datasets/CodeSearchNet/java',
        help="The folder where some training data in the format for the pytorch model is")

def get_pytorch_data(data_dir):
    examples = ptrun.read_examples(data_dir)
    return None

def get_mxnet_data(data_dir):
    tokenizer = RobertaTokenizer.from_pretrained('microsoft/codebert-base')
    stages = ['train', 'test']
    train_data = conv.get_stage_data('train', tokenizer, data_dir, True)
    test_data = conv.get_stage_data('test', tokenizer, data_dir, True)
    return train_data, test_data

if __name__ == '__main__':
    args = parse_args()
    pytorch_data = get_pytorch_data(args.data_dir)
    mxnet_data = get_mxnet_data(args.data_dir)