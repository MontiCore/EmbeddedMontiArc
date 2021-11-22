/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

// import de.serwth.emadl.Optimizer;
import Optimizer;

schema DDPG extends ReinforcementLearning {

    critic: component
    soft_target_update_rate: Q
    critic_optimizer: optimizer_type
}