/* (c) https://github.com/MontiCore/monticore */
package de.serwth.emadl;

/*
* Schema definition for optimizer.
*/
schema ImportedOptimizer {

    optimizer {
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
            fixed,
            step,
            exp,
            inv,
            poly,
            sigmoid;
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
}