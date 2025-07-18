/* (c) https://github.com/MontiCore/monticore */

schema PortOfVectorTypeIsChecked {

    reference-model: ddpg16.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}