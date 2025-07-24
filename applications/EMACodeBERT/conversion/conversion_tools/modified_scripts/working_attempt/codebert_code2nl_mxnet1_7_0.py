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

from mxnet import gluon
from codebert_models import TransformerDecoder
from transformers.models.roberta import RobertaTokenizer
from codebert_models import Seq2Seq
from cnnarch2gluon_adamw import AdamW
from itertools import cycle
import codebert_hyper_params as hp
import codebert_code2nl_bleu as bleu
import convert_code2nl_data_mxnet1_7_0 as conv
import mxnet as mx
import gluonnlp as nlp
import numpy as np
import argparse
import h5py

def get_decoder(test_run, ctx):
    decoder_hparams = hp.get_decoder_hparams()
    # gluon TransformerDecoder does a positional encoding before input, does codebert do the same thing?
    # gluonnlp might not do it if position_weight is none?
    # torch model constructs the ffn in the forward call, gluon uses a custom layer for it PositionwiseFFN
    decoder = TransformerDecoder(
        attention_cell=decoder_hparams['attention_cell'], 
        num_layers=decoder_hparams['num_layers'], 
        units=decoder_hparams['units'], 
        hidden_size=decoder_hparams['hidden_size'],
        max_length=decoder_hparams['max_length'], 
        num_heads=decoder_hparams['num_heads'], 
        scaled=decoder_hparams['scaled'], 
        scale_embed=decoder_hparams['scale_embed'], 
        norm_inputs=decoder_hparams['norm_inputs'],
        dropout=decoder_hparams['dropout'], 
        use_residual=decoder_hparams['use_residual'], 
        output_attention=decoder_hparams['output_attention'], 
        weight_initializer=decoder_hparams['weight_initializer'],
        bias_initializer=decoder_hparams['bias_initializer'], 
        prefix=decoder_hparams['prefix'], 
        params=decoder_hparams['params']
    )
    return decoder

def get_seq2seq(sym_file, wt_file, esym_file, ewt_file, ctx, test_run):
    embedding = load_codebert_block(esym_file, ewt_file, ctx)
    encoder = load_codebert_block(sym_file, wt_file, ctx)
    decoder = get_decoder(test_run, ctx)
    seq2seq_hparams = hp.get_seq2seq_hparams()
    training_params = hp.get_training_hparams(test_run)
    seq2seq = Seq2Seq(
        # params taken from cmd line args in codebert repo and roberta config
        embedding, encoder, decoder,
        hidden_size=seq2seq_hparams['hidden_size'],
        vocab_size=seq2seq_hparams['vocab_size'],
        beam_size=training_params['beam_size'], 
        max_length=training_params['max_target_length'], 
        sos_id=seq2seq_hparams['sos_id'],
        eos_id=seq2seq_hparams['eos_id']
    )
    return seq2seq

def load_codebert_block(symbol_file, weight_file, context):
    input_names = ['data0', 'data1']
    return gluon.nn.SymbolBlock.imports(symbol_file, input_names, weight_file, ctx=context)

def get_seqs_from_batch(batch, ctx):
    source_ids = gluon.utils.split_and_load(batch.data[0], ctx_list=ctx, even_split=False)
    source_masks = gluon.utils.split_and_load(batch.data[1], ctx_list=ctx, even_split=False)
    target_ids = gluon.utils.split_and_load(batch.label[0], ctx_list=ctx, even_split=False)
    target_masks = gluon.utils.split_and_load(batch.label[1], ctx_list=ctx, even_split=False)
    return source_ids, source_masks, target_ids, target_masks

def valid_lengths_to_ones(masks, max_length, ctx):
    new_masks = []
    int_masks = masks.asnumpy().astype(np.int32)
    for length in int_masks:
        ones = np.ones(length).astype(np.int32).tolist()
        if max_length > length:
            zeros = np.zeros(max_length-length).astype(np.int32).tolist()
            new_masks.append(ones + zeros)
        else: 
            new_masks.append(ones)
    return mx.nd.array(new_masks, ctx=ctx)

def train_model(seq2seq, train_data, ctx, test_run, num_gpus):
    train_hparams = hp.get_training_hparams(test_run)
    batch_size = train_hparams['batch_size']
    train_steps = train_hparams['train_steps']
    max_target_length = train_hparams['max_target_length']
    seq2seq.initialize(ctx=ctx)
    seq2seq.hybridize()
    loss = mx.gluon.loss.SoftmaxCrossEntropyLoss() # TODO parameters? e.g. sparse_label
    # lr is defined in the call to run codebert, epsilon is left as default in the run script, beta1 and 2 are the pytorch default
    optimizer = AdamW(
        learning_rate = train_hparams['learning_rate'], 
        beta1 = 0.9, 
        beta2 = 0.999, 
        epsilon = train_hparams['adam_epsilon']
    )
    trainer = mx.gluon.Trainer(seq2seq.collect_params(), optimizer=optimizer)
    if (test_run):
        print('Doing test run with subset of data...', flush=True)
    else:
        print('Training full model, make sure you exported the correct data!', flush=True)
    print('Training steps {}'.format(train_steps), flush=True)
    print('Batch size {}'.format(batch_size), flush=True)
    print('Num samples {}'.format(train_data.num_data), flush=True)
    print('Training model...', flush=True)
    all_shift_logits = []
    batch_losses = []
    total_loss = 0
    train_data = cycle(train_data)
    for bid in range(train_steps):
        batch = next(train_data)
        source_ids, source_masks, target_ids, target_masks = get_seqs_from_batch(batch, ctx)
        with mx.autograd.record():
            losses = []
            for s_id, s_msk, tgt_id, tgt_msk in zip(source_ids, source_masks, target_ids, target_masks):
                    lm_logits = seq2seq(s_id, s_msk, tgt_id, tgt_msk)
                    # print(50*"*" + "lm_logits")
                    # print(lm_logits)
                    # print(50*"*" + "tgt_mask")
                    # print(tgt_msk)
                    # print(50*"*" + "target_ids")
                    # print(tgt_id)
                    # drop the start of sentence mask tokens? - mb
                    loss_tgt_mask = valid_lengths_to_ones(tgt_msk, max_target_length, lm_logits.ctx)
                    # print(50*"*" + "loss_tgt_mask")
                    # print(loss_tgt_mask)
                    active_loss = loss_tgt_mask[..., 1:].reshape(-1)
                    shift_labels = tgt_id[..., 1:]
                    # print(50*"*"+ "active_loss")
                    # print(active_loss)
                    # print(50*"*"+ "shift_labels")
                    # print(shift_labels)
                    # Shift so that tokens < n predict n
                    # shape is (batch_size, target_seq_len, vocab_size)
                    # so for every batch, and every word in each batch
                    # we have unnormalized probablilities of next word (50265 words)
                    shift_logits = lm_logits[..., :-1, :]
                    # print(50*"*" + "shift_logits")
                    # print(shift_logits)
                    all_shift_logits.append(shift_logits.copyto(mx.cpu()).detach())
                    newDim = shift_logits.shape[-1]
                    X = mx.nd.contrib.boolean_mask(shift_logits.reshape(-1, newDim), active_loss)
                    y = mx.nd.contrib.boolean_mask(shift_labels.reshape(-1), active_loss)
                    # print(50*"*"+ "X")
                    # print(X)
                    # print(50*"*"+ "y")
                    # print(y)
                    # mean loss output because torch_loss(X,y) == mxnet_loss(X,y).mean()
                    l = loss(X, y).mean()
                    # append to list to later take average over different gpu outputs
                    losses.append(l)
            # cant average over the gpu slices like in the pytorch script, not sure if better way
            for l in losses:
                l.backward()
        # torch script loss func outputs the mean of the batch torch_loss(X,y) == mxnet_loss(X,y).mean()
        # we sum over the losses on each device and then divide that by the current step or batch number
        # TODO with multiple devices e.g. 2 gpus do we need to divinde total_loss by num_gpus?
        total_loss += sum([l.asscalar() for l in losses])
        batch_loss = (total_loss/max(1, num_gpus))/(bid+1)
        batch_losses.append(batch_loss)
        print('Train Batch {}/{} Loss {}'.format(
            bid+1, train_steps, batch_loss
        ), flush=True)
        trainer.step(batch_size)
    return seq2seq, all_shift_logits, batch_losses

def test_model_w_data(seq2seq, test_data, ctx, test_run):
    print('Testing with Model...', flush=True)
    train_hparams = hp.get_training_hparams(test_run)
    batch_size = train_hparams['batch_size']
    num_batches = test_data.num_data//batch_size
    preds = []
    probs = []
    for bid in range(num_batches):
        batch = next(test_data)
        print('Test Batch {}/{}'.format(bid+1, num_batches), flush=True)
        source_ids, source_masks, target_ids, _ = get_seqs_from_batch(batch, ctx)
        for s_id, s_msk, tgt_id in zip(source_ids, source_masks, target_ids):
            pred, prob = seq2seq(s_id, s_msk)
            preds.append((pred, tgt_id.copyto(mx.cpu()).detach()))
            probs.append(prob)
    return preds, probs

# we need a tokenizer to quanitify
def format_for_bleu(tokenizer, outputs):
    formatted_preds = []
    formatted_actuals = []
    # go through batch outputs
    counter = 0
    for preds, actuals in outputs:
        new_preds = []
        # go through sequences in batch
        for idx, pred in enumerate(preds):
            actual = actuals[idx].asnumpy().tolist()
            # TODO not sure why we take the first entry, maybe this is the best beam?
            t = pred[0].asnumpy().tolist()
            # TODO is this removing spaces or padding? was in the original script
            if 0 in t:
                t = t[:t.index(0)]
            pred_text = tokenizer.decode(t, clean_up_tokenization_spaces=False)
            actual_text = tokenizer.decode(actual, clean_up_tokenization_spaces=False)
            formatted_preds.append(str(counter)+'\t'+pred_text)
            formatted_actuals.append(str(counter)+'\t'+actual_text)
            counter += 1
    return formatted_preds, formatted_actuals

def compute_bleu(file_name, res):
    pred, actual = res
    actual_map, pred_map = bleu.computeMaps(pred, actual) 
    score = round(bleu.bleuFromMaps(actual_map, pred_map)[0], 2)
    print('{} {}'.format('Bleu-4 Score', str(score)), flush=True)

def get_data(args, fname):
    train_hparams = hp.get_training_hparams(args.test_run)
    data_file = '{}/{}'.format(args.data_dir, fname)
    data_iter = conv.get_data_iterator(
        ['source_ids', 'source_masks'], ['target_ids', 'target_masks'],
        True, train_hparams['batch_size'], h5py.File(data_file, 'r'))
    return data_iter

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test_run', action='store_true', 
        help='Run the training and testing on a very small subset of the data')
    parser.add_argument('--data_dir', default='./codebert_gluon/data', type=str,
        help='The folder where the processed training, validation and test data is.')
    parser.add_argument("--symbol_file", default='./codebert_gluon/model/codebert-symbol.json', type=str,
        help="Symbol file from the pretrained model output by the conversion script")
    parser.add_argument("--weight_file", default='./codebert_gluon/model/codebert-0000.params', type=str,
        help="Weight file from the pretrained model output by the conversion script")
    parser.add_argument("--embed_symbol_file", default='./codebert_gluon/model/codebert_embedding-symbol.json', type=str,
        help="Symbol file from the pretrained embed output by the conversion script")
    parser.add_argument("--embed_weight_file", default='./codebert_gluon/model/codebert_embedding-0000.params', type=str,
        help="Weight file from the pretrained embed output by the conversion script")
    parser.add_argument("--num_gpus", default=2, type=int,
        help="Number of gpus to train on, set to 0 for cpu training")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()

    if args.num_gpus < 1:
        ctx = [mx.cpu()]
    else:
        ctx = [mx.gpu(i) for i in range(args.num_gpus)]

    seq2seq = get_seq2seq(
        args.symbol_file, args.weight_file, 
        args.embed_symbol_file, args.embed_weight_file, 
        ctx, args.test_run
    )

    train_data = get_data(args, 'train.h5')
    model,_,_  = train_model(seq2seq, train_data, ctx, args.test_run, args.num_gpus)
    tokenizer = RobertaTokenizer.from_pretrained('microsoft/codebert-base')

    test_data = get_data(args, 'test.h5')
    preds,_ = test_model_w_data(model, test_data, ctx, args.test_run)
    formatted = format_for_bleu(tokenizer, preds)
    compute_bleu('test.h5', formatted)