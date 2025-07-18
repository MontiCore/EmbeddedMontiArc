/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

import Optimizer;

schema SingleSchemaLinksDefinition {

    learning_method: schema {
        supervised -> SupervisedLearning,
        reinforcement -> ReinforcementLearning,
        gan -> GANLearning;
    }

    context: enum {
        cpu,
        gpu;
    }
    optimizer: optimizer_type
    actor_optimizer: optimizer_type
}