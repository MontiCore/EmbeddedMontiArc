/* (c) https://github.com/MontiCore/monticore */
import Loss;

schema DQN extends ReinforcementLearning {

    reference-model: referencemodels.dqn.DQN, referencemodels.dqn.DQNGym, referencemodels.dqn.DQNGymPolicyReward
    use_fix_target_network: B
    target_network_update_interval: N
    use_double_dqn: B
    loss: loss_type
    strategy: strategy_type

    strategy_type {
        values: epsgreedy;

        epsilon: Q
        min_epsilon: Q
        epsilon_decay_start: Z
        epsilon_decay_method: enum {
            linear, no;
        }
        epsilon_decay_per_step = false: B
        epsilon_decay: Q
    }
}