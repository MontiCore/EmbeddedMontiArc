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
    /* loss_weights: Q^3 */
    preprocessing_name: component
    use_teacher_forcing: B
    save_attention_image: B
    eval_train: B
    loss: loss_function

    loss_function {
        values:
            l1,
            l2,
            epe,
            log_cosh,
            sigmoid_binary_cross_entropy,
            huber,
            cross_entropy,
            softmax_cross_entropy,
            softmax_cross_entropy_ignore_indices,
            dice,
            hinge,
            squared_hinge,
            logistic,
            kullback_leibler;

        define huber {
            rho: Q
        }

        define cross_entropy {
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
        }

        define softmax_cross_entropy {
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
            from_logits: B
        }

        define softmax_cross_entropy_ignore_indices {
            sparse_label: B
            loss_axis: Z
            batch_axis: Z
            from_logits: B
            ignore_indices: Z
        }

        define hinge {
            margin: Q
        }

        define hinge {
            label_format: string
        }

        define kullback_leibler {
            from_logits: B
        }
    }
}