/* (c) https://github.com/MontiCore/monticore */

schema InputAndOutputPortTypesAndRangesDontMatch {

    reference-model: ddpg14.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}