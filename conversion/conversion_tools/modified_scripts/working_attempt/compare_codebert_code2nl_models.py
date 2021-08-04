import codebert_models as mxmod
import codebert_code2nl.model as ptmod
import convert_code2nl_data_mxnet1_7_0 as conv
import codebert_code2nl.run as ptrun
import codebert_hyper_params as hyp

from transformers.models.roberta import RobertaTokenizer

import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", default='/home/makua/Documents/Datasets/CodeSearchNet/java',
        help="The folder where some training data in the format for the pytorch model is")


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

if __name__ == '__main__':
    args = parse_args()
    pt_train = get_pytorch_dataloader(args.data_dir, 'dev', 'train.jsonl')
    pt_test = get_pytorch_dataloader(args.data_dir, 'test', 'test.jsonl')
    mx_train, mx_test = get_mxnet_data(args.data_dir)