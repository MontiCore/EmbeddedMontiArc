/* (c) https://github.com/MontiCore/monticore */

schema MultipleReferenceModelsFail {

    reference-model: ddpg13.DDPG, ddpg13.DDPGWithReward

    learning_method: enum {
        reinforcement;
    }

    rl_algorithm: enum {
        ddpg;
    }

    environment: environment_type

    environment_type {
        values: ros_interface;

        define ros_interface {
            state: string
            action: string
            reset: string
            terminal: string
            reward: string
        }
    }
}