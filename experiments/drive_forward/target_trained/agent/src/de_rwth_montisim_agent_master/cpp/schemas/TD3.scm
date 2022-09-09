/* (c) https://github.com/MontiCore/monticore */
import Optimizer;

schema TD3 extends DDPG {

    policy_noise: Q
    noise_clip: Q
    policy_delay: N
    noise_variance: Q
}