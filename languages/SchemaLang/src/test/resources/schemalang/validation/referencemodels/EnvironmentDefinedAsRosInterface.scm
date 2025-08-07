/* (c) https://github.com/MontiCore/monticore */

schema EnvironmentDefinedAsRosInterface {

    reference-model: ddpg8.DDPG

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