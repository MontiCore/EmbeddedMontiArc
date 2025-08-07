def get_bertenc_hparams(test): 
    return {
        'num_layers': 12,
        'units': 768,
        'hidden_size': 3072,
        'max_length': 512,
        'num_heads': 12,
        'dropout': 0.1,
        'output_attention': False,
        'output_all_encodings': True if test else False,
        'weight_initializer': None,
        'bias_initializer': 'zeros',
        'prefix': None,
        'params': None,
        'activation': 'gelu',
        'layer_norm_eps': 1e-5
    }

def get_bert_hparams():
    return {
        'vocab_size': 50265,
        'token_type_vocab_size': 1,
        'units': 768,
        'embed_size': 768,
        'embed_initializer': None,
        'word_embed': None,
        'token_type_embed': None,
        'use_pooler': False,
        'use_decoder': False,
        'use_classifier': False,
        'use_token_type_embed': True,
        'prefix': "codebert0_",
        'params': None
    }

def get_decoder_hparams():
    # put together by looking at the torch decoder
    return {
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

def get_seq2seq_hparams():
    return {
        'hidden_size': 768,
        'vocab_size': 50265, # TODO correct vocab size? or should it be -2/+2?
        'sos_id': 0,
        'eos_id': 2
    }

def get_training_hparams(test_run):
    if test_run:
        return {
            'learning_rate': 5e-5,
            'adam_epsilon': 1e-8,
            'max_source_length': 32,
            'max_target_length': 16,
            'limit_train_samples': 128,
            'limit_valid_samples': 64,
            'limit_test_samples': 64,
            'train_steps': 16,
            'batch_size': 8,
            'beam_size': 10
        }
    else:
        return {
            'learning_rate': 5e-5,
            'adam_epsilon': 1e-8,
            'max_source_length': 256,
            'max_target_length': 128,
            'limit_train_samples': -1,
            'limit_valid_samples': -1,
            'limit_test_samples': -1,
            'train_steps': 50000,
            'batch_size': 64,
            'beam_size': 10
        }