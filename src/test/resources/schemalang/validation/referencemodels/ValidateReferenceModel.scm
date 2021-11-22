/* (c) https://github.com/MontiCore/monticore */

schema ValidateReferenceModel {

    reference-model: ddpg1.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}