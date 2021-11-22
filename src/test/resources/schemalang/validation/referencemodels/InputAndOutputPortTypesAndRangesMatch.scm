/* (c) https://github.com/MontiCore/monticore */

schema InputAndOutputPortTypesAndRangesMatch {

    reference-model: ddpg6.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}