/* (c) https://github.com/MontiCore/monticore */

schema RequiredComponentMissing {

    reference-model: ddpg5.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}