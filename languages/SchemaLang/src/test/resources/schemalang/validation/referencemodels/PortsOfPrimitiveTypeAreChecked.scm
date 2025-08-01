/* (c) https://github.com/MontiCore/monticore */

schema PortsOfPrimitiveTypeAreChecked {

    reference-model: ddpg15.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}