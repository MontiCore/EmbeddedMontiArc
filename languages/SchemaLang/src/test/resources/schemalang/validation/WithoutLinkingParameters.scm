/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

import Optimizer;

schema WithoutLinkingParameters {

    learning_method: schema {
        supervised,
        reinforcement,
        gan;
    }

    context: enum {
        cpu,
        gpu;
    }
    optimizer: optimizer_type
    actor_optimizer: optimizer_type
}