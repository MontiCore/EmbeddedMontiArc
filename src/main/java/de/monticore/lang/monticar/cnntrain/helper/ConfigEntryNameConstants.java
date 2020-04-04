/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain.helper;

public class ConfigEntryNameConstants {
    public static final String LEARNING_METHOD = "learning_method";
    public static final String EVAL_METRIC = "eval_metric";
    public static final String NUM_EPISODES = "num_episodes";
    public static final String DISCOUNT_FACTOR = "discount_factor";
    public static final String NUM_MAX_STEPS = "num_max_steps";
    public static final String TARGET_SCORE = "target_score";
    public static final String TRAINING_INTERVAL = "training_interval";
    public static final String USE_FIX_TARGET_NETWORK = "use_fix_target_network";
    public static final String TARGET_NETWORK_UPDATE_INTERVAL = "target_network_update_interval";
    public static final String SNAPSHOT_INTERVAL = "snapshot_interval";
    public static final String AGENT_NAME = "agent_name";
    public static final String USE_DOUBLE_DQN = "use_double_dqn";
    public static final String LOSS = "loss";
    public static final String RL_ALGORITHM = "rl_algorithm";
    public static final String REPLAY_MEMORY = "replay_memory";
    public static final String ENVIRONMENT = "environment";
    public static final String START_TRAINING_AT = "start_training_at";
    public static final String SOFT_TARGET_UPDATE_RATE = "soft_target_update_rate";
    public static final String EVALUATION_SAMPLES = "evaluation_samples";
    public static final String POLICY_NOISE = "policy_noise";
    public static final String NOISE_CLIP = "noise_clip";
    public static final String POLICY_DELAY = "policy_delay";

    public static final String ENVIRONMENT_REWARD_TOPIC = "reward_topic";
    public static final String ENVIRONMENT_ROS = "ros_interface";
    public static final String ENVIRONMENT_GYM = "gym";

    public static final String STRATEGY = "strategy";
    public static final String STRATEGY_OU = "ornstein_uhlenbeck";
    public static final String STRATEGY_OU_MU = "mu";
    public static final String STRATEGY_OU_THETA = "theta";
    public static final String STRATEGY_OU_SIGMA = "sigma";
    public static final String STRATEGY_GAUSSIAN = "gaussian";
    public static final String STRATEGY_EPSGREEDY = "epsgreedy";
    public static final String STRATEGY_EPSDECAY = "epsdecay";

    public static final String CRITIC = "critic";

    public static final String DISCRIMINATOR_NAME = "discriminator_name";
    public static final String QNETWORK_NAME = "qnet_name";
    public static final String PREPROCESSING_NAME = "preprocessing_name";
    public static final String NOISE_DISTRIBUTION = "noise_distribution";
    public static final String CONSTRAINT_DISTRIBUTION = "constraint_distributions";
    public static final String CONSTRAINT_LOSS = "constraint_losses";
    public static final String DISCRIMINATOR_OPTIMIZER = "discriminator_optimizer";
    public static final String K_VALUE = "k_value";
    public static final String GENERATOR_LOSS = "generator_loss";
    public static final String GENERATOR_TARGET_NAME = "generator_target_name";
    public static final String NOISE_INPUT = "noise_input";
    public static final String GENERATOR_LOSS_WEIGHT = "generator_loss_weight";
    public static final String DISCRIMINATOR_LOSS_WEIGHT = "discriminator_loss_weight";
    public static final String PRINT_IMAGES = "print_images";
}

