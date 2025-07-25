/* (c) https://github.com/MontiCore/monticore */

schema ValidateConnections {

    reference-model: ddpg2.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}