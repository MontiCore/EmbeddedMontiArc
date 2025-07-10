/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema General {

    learning_method: enum {
        SUPERVISED_LEARNING,
        REINFORCEMENT_LEARNING,
        GAN;
    }
    context: enum {
        CPU,
        GPU;
    }
    optimizer: optimizer

    optimizer {
        values:
            sgd,
            adam,
            adamw,
            rmsprop,
            adagrad,
            nesterov;

        learning_rate_minimum: Q
        learning_rate: Q
        weight_decay: Q
        learning_rate_decay: Q
        learning_rate_policy: enum {
            FIXED,
            STEP,
            EXP,
            INV,
            POLY,
            SIGMOID;
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

        define nesterov {
            momentum: Q
        }
    }
}
