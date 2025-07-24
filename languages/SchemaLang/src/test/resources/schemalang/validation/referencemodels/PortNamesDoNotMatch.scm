/* (c) https://github.com/MontiCore/monticore */

schema PortNamesDoNotMatch {

    reference-model: ddpg7.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}