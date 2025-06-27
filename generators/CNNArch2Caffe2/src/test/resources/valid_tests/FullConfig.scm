/* (c) https://github.com/MontiCore/monticore */

schema FullConfig {

    context: enum {
        cpu,
        gpu;
    }

    num_epoch: N0
    batch_size: N0
    load_checkpoint: B
    normalize: B
    optimizer: optimizer_type
    eval_metric: eval_metric_type

    optimizer_type {
        values:
            rmsprop;

        define rmsprop {
            learning_rate: Q
            learning_rate_minimum: Q
            rescale_grad: Q
            clip_gradient: Q
            clip_weights: Q
            centered: B
            weight_decay: Q
            learning_rate_decay: Q
            learning_rate_policy: enum {
                step;
            }
            step_size: N0
            gamma1: Q
            gamma2: Q
            epsilon: Q
        }
    }

    eval_metric_type {
        values:
            mse;
    }

    loss_type {
        values:
            cross_entropy;
    }
}