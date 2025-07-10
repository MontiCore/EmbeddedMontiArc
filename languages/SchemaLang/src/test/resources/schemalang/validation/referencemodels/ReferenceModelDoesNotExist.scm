/* (c) https://github.com/MontiCore/monticore */

schema ReferenceModelDoesNotExist {

    reference-model: referencemodel.NotExists

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }
}