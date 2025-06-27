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
    optimizer_type {
               values:
                   sgd,
                   adam,
                   adamw;

               /*
               * Parameters common to all optimizer algorithms.
               */
               learning_rate: Q
               weight_decay: Q
               learning_rate_policy: enum {
               fixed, step, exp, inv, poly, sigmoid;
               }

               define sgd {
                   momentum: Q
               }

               define adam {
                   beta1: Q
                   beta2: Q
                   epsilon: Q
               }
           }

    loss_type {
            values:
                cross_entropy,
                kullback_leibler;

            define huber {
                rho: Q
            }

            define cross_entropy {
                sparse_label: B
                loss_axis: Z
                batch_axis: Z
            }


        }

    eval_metric_type {
            values:
                rmse,
                accuracy,
                accuracy_ignore_label;

            define accuracy_ignore_label {
                axis: Z
                metric_ignore_label: Z
            }
      }

    eval_metric: eval_metric_type
    optimizer: optimizer_type
    loss:loss_type
    // primitive types
    batch_size: N1
    num_epoch: N
    checkpoint_period = 5: N
    preprocessing_name: component
    load_pretrained: B
    log_period: N
    shuffle_data: B
}