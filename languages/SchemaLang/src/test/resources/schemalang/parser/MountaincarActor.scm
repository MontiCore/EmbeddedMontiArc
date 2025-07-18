/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema MountaincarActor {

    context: enum {
        cpu,
        gpu;
    }

    learning_method: enum {
        supervised,
        reinforcement,
        gan;
    }

    rl_algorithm: enum {
        dqn,
        ddpg,
        td3;
    }

    critic: Component
    num_episodes: N
    target_score: Q
    discount_factor: Q
    num_max_steps: N
    training_interval: N

    use_fix_target_network: B
    target_network_update_interval: N
    snapshot_interval: N
    use_double_dqn: B

    environment: environment
    loss: loss
    replay_memory: replay_memory
    strategy: strategy
    optimizer: optimizer
    actor_optimizer: optimizer
    critic_optimizer: optimizer

    environment {
        values:
            gym;

        define gym {
            name: S
        }
    }

    loss {
        values:
            huber;
    }

    replay_memory {
        values:
            buffer;

        define buffer {
            memory_size: N
            sample_size: N
        }
    }

    strategy {
        values:
            epsgreedy,
            ornstein_uhlenbeck;

        define epsgreedy {
            epsilon: Q
            min_epsilon: Q
            epsilon_decay_method: enum {
                linear;
            }
            epsilon_decay: Q
        }

        define ornstein_uhlenbeck {
            epsilon: Q
            min_epsilon: Q
            epsilon_decay_method: enum {
                linear;
            }
            epsilon_decay: Q
            mu: Q*
            theta: Q*
            sigma: Q*
        }
    }

    optimizer {
        values:
            adam,
            rmsprop;

        define adam {
            learning_rate: Q
            learning_rate_policy: enum {
                fixed,
                step,
                exp,
                inv,
                poly,
                sigmoid;
            }
            weight_decay: Q
            epsilon: Q
            beta1: Q
            beta2: Q
        }

        define rmsprop {
            learning_rate: Q
        }
    }
}