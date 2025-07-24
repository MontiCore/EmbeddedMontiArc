/* (c) https://github.com/MontiCore/monticore */

schema CriticComponentDefinedInSchema {

    reference-model: ddpg9.DDPG

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }

    critic: component // this should not be allowed, because critic is defined in reference model ddpg8.DDPG
}