/* (c) https://github.com/MontiCore/monticore */
//import SchemaWithObjectTypeEntry;

schema Supervised {

    network_type: schema {
        gnn;
    }

    //enums
     context: enum {
            cpu, gpu;
        }

    //object type
    loss_type {
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

            define softmax_cross_entropy_ignore_label {
                sparse_label: B
                loss_axis: Z
                batch_axis: Z
                from_logits: B
                loss_ignore_label: Z
            }

            define hinge {
                margin: Q
            }

            define logistic {
                label_format: string
            }

            define kullback_leibler {
                from_logits: B
            }
        }

    eval_metric_type {
            values:
                rmse,
                accuracy_ignore_label,
                bleu;
             //batchy: N1
            define accuracy_ignore_label {
                axis: Z
                metric_ignore_label: Z
            }

            define bleu {
                exclude: Z*
            }
        }

     optimizer_type {
             values:
                 sgd,
                 adam,
                 adamw,
                 rmsprop,
                 adagrad,
                 nag,
                 adadelta;

             /*
             * Parameters common to all optimizer algorithms.
             */
             learning_rate: Q
             learning_rate_minimum: Q
             learning_rate_decay: Q
             weight_decay: Q
             learning_rate_policy: enum {
                 fixed, step, exp, inv, poly, sigmoid;
             }
             rescale_grad: Q
             clip_gradient: Q
             step_size: Z

             define sgd {
                 momentum: Q
             }

             define adam {
                 beta1: Q
                 beta2: Q
                 epsilon: Q
             }

             define adamw {
                 beta1: Q
                 beta2: Q
                 epsilon: Q
             }

             define rmsprop {
                 epsilon: Q
                 gamma1: Q
                 gamma2: Q
                 centered: B
                 clip_weights: Q
                 rho: Q
             }

             define adagrad {
                 epsilon: Q
             }

             define nag {
                 momentum: Q
             }

             define adadelta {
                 epsilon: Q
                 rho: Q
             }
         }
    // primitive types
    eval_metric: eval_metric_type
    optimizer: optimizer_type
    batch_size: N1
    num_epoch: N
    checkpoint_period = 5: N
    preprocessing_name: component
    load_pretrained: B
    log_period: N
    shuffle_data: B
}