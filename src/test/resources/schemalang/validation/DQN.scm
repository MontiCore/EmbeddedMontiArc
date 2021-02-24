/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

import Loss;

schema DQN extends Reinforcement {

    use_fix_target_network: B
    target_network_update_interval: N
    use_double_dqn: B
    loss: loss_type
}