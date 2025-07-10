/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema SupervisedLearning extends General {
    num_epoch = 100000: N
    batch_size = 64: N
    load_checkpoint: B
    checkpoint_period: Z
    load_pretrained: B
    log_period: Z
    normalize = true: B
    loss_weights: DoubleVectorValue
    preprocessing_name: ComponentName
    use_teacher_forcing: B
    save_attention_image: B
    eval_train: B

    eval_metric {
        accuracy,
        cross_entropy,
        f1,
        mae,
        mse,
        perplexity,
        rmse,
        top_k_accuracy,
        accuracy_ignore_label {
            axis: Z
            metric_ignore_label: Z
        },
        bleu {
            exclude: Z*
        },
    }



    loss {
        l1 {},
        l2 {},
        epe {},
        log_cosh {},
        sigmoid_binary_cross_entropy {},
        huber {
            rho: Q
        },
        cross_entropy {
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        },
        softmax_cross_entropy {
            from_logits: B
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        }
        softmax_cross_entropy_ignore_indices {
            ignore_indices: Z
            from_logits: B
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        }
        dice_loss {
            from_logits: B
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        }
        softmax_cross_entropy_ignore_label {
            loss_ignore_label: Z
            from_logits: B
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        },
        hinge {
            margin: Q
        }
        squared_hinge {
            margin: Q
        }
        logistic {
            label_format: S
        }
        kullback_leibler {
            from_logits:B
        }
    }
}