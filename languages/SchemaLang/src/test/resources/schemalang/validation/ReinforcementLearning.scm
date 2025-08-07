/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema ReinforcementLearning extends General {

    rl_algorithm: schema {
        ddpg -> DDPG,
        dqn -> DQN,
        td3 -> TD3;
    }

    num_episodes: N1
    discount_factor: Q(0:1)
    num_max_steps: N
    target_score: Q
    training_interval: N
    start_training_at: N
    evaluation_samples: N
    snapshot_interval: N
    agent_name: string
    reward_function: component

    environment: environment_type
    replay_memory: replay_memory_type
    strategy: strategy_type

    environment_type {
        values:
            gym,
            ros_interface;

        define gym {
            name: string
        }

        define ros_interface {
            state_topic: string
            action_topic: string
            reset_topic: string
            terminal_state_topic: string
            reward_topic: string
        }
    }

    replay_memory_type {
        values:
            buffer;

        define buffer {
            memory_size: N
            sample_size: N
        }
    }

    strategy_type {
        values:
            epsgreedy,
            ornstein_uhlenbeck,
            gaussian;

        epsilon: Q
        min_epsilon: Q
        epsilon_decay_start: Z
        epsilon_decay_method: enum {
            linear,
            no;
        }
        epsilon_decay_per_step: B
        epsilon_decay: Q

        define ornstein_uhlenbeck {
            mu: Q*
            theta: Q*
            sigma: Q*
        }

        define gaussian {
            noise_variance: Q
        }
    }
}