/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema ComplexProperty {

    strategy_epsgreedy: strategy
    strategy_gaussian: strategy
    strategy_ornstein_uhlenbeck: strategy

    strategy {
        values:
            epsgreedy;

        epsilon: Q
        min_epsilon: Q
        epsilon_decay_start: Z
        epsilon_decay_method: enum {
            linear,
            no;
        }
        epsilon_decay_per_step: B
        epsilon_decay: Q
    }
}