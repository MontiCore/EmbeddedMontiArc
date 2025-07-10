/* (c) https://github.com/MontiCore/monticore */

schema InputAndOutputPortTypesAndRangesDiffer {

    reference-model: ddpg4.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}