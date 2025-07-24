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
    optimizer: Class<Optimizer>

    class Optimizer {
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
    }

    class SGDOptimizer extends Optimizer {
        momentum: Q
    }

    class AdamOptimizer extends Optimizer {
        beta1: Q
        beta2: Q
        epsilon: Q
    }

    class AdamWOptimizer extends Optimizer {
        beta1: Q
        beta2: Q
        epsilon: Q
    }

    class RmsPropOptimizer extends Optimizer {
        epsilon: Q
        gamma1: Q
        gamma2: Q
        centered: B
        clip_weights: Q
        rho: Q
    }

    class AdaGradOptimizer extends Optimizer {
        epsilon: Q
    }

    class NesterovOptimizer extends Optimizer {
        momentum: Q
    }

    class NesterovOptimizer extends Optimizer {
        momentum: Q
    }
}
