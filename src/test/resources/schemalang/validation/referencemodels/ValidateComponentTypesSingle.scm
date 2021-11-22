/* (c) https://github.com/MontiCore/monticore */

schema ValidateComponentTypesSingle {

    reference-model: ddpg3.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}