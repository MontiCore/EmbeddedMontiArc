/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

import de.serwth.emadl.Optimizer;

schema ImportedSchema {

    critic: component
    soft_target_update_rate: Q
    policy_noise: Q
    noise_clip: Q
    policy_delay: N
    noise_variance: Q
    critic_optimizer: optimizer
}