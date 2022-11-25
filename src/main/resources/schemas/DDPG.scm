/* (c) https://github.com/MontiCore/monticore */
import Optimizer;

schema DDPG extends Reinforcement {

    reference-model: referencemodels.ddpg.DDPG, referencemodels.ddpg.DDPGWithoutRewardComponent, referencemodels.ddpg.DDPGGymEnvironment
    soft_target_update_rate: Q
    critic_optimizer: optimizer_type
    strategy: strategy_type

    strategy_type {
        values: ornstein_uhlenbeck, gaussian;

        epsilon: Q
        min_epsilon: Q
        epsilon_decay_start: Z
        epsilon_decay_method: enum {
            linear, no;
        }
        epsilon_decay_per_step = false: B
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