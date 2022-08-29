/* (c) https://github.com/MontiCore/monticore */
//import SchemaWithObjectTypeEntry;

schema Supervised {

    reference-model: referencemodels.ddpg.DDPG2
    network_type: schema {
        gnn;
    }

    //enums
     context: enum {
            cpu, gpu;
        }

    //object type
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