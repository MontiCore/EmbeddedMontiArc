/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.training;

public class TrainingParameterConstants {

    /*
     * Learning paradigms
     */
    public static final String SUPERVISED = "supervised";
    public static final String REINFORCEMENT = "reinforcement";
    public static final String GAN = "gan";
    public static final String VAE = "vae";

    /*
     * Optimizers
     */
    public static final String OPTIMIZER_ADA_DELTA = "adadelta";
    public static final String OPTIMIZER_NESTEROV = "nag";
    public static final String OPTIMIZER_ADAMW = "adamw";

    /*
     * Hyper parameters
     */
    public static final String LEARNING_RATE_MINIMUM = "learning_rate_minimum";
    public static final String RESCALE_GRAD = "rescale_grad";
    public static final String CLIP_GRADIENT = "clip_gradient";
    public static final String GAMMA2 = "gamma2";
    public static final String CENTERED = "centered";
    public static final String CLIP_WEIGHTS = "clip_weights";
    public static final String BATCH_SIZE = "batch_size";
    public static final String NUM_EPOCH = "num_epoch";
    public static final String LOAD_CHECKPOINT = "load_checkpoint";
    public static final String CHECKPOINT_PERIOD = "checkpoint_period";
    public static final String LOG_PERIOD = "log_period";
    public static final String LOAD_PRETRAINED = "load_pretrained";
    public static final String NORMALIZE = "normalize";
    public static final String CONTEXT = "context";
    public static final String CLEANING = "cleaning";
    public static final String DATA_IMBALANCE = "data_imbalance";
    public static final String DATA_SPLITTING = "data_splitting";
    public static final String SHUFFLE_DATA = "shuffle_data";
    public static final String CLIP_GLOBAL_GRAD_NORM = "clip_global_grad_norm";
    public static final String USE_TEACHER_FORCING = "use_teacher_forcing";
    public static final String SAVE_ATTENTION_IMAGE = "save_attention_image";
    public static final String EVAL_TRAIN = "eval_train";
    public static final String OPTIMIZER = "optimizer";
    public static final String RETRAINING_TYPE = "retraining_type";

    public static final String RETRAINING_OPTIMIZER = "retraining_optimizer";
    public static final String ACTOR_OPTIMIZER = "actor_optimizer";
    public static final String CRITIC_OPTIMIZER = "critic_optimizer";
    public static final String ONNX_EXPORT = "onnx_export";

    // Reinforcement
    public static final String REWARD_FUNCTION = "reward";
    public static final String POLICY_FUNCTION = "policy";
    public static final String DQN = "dqn";
    public static final String DDPG = "ddpg";
    public static final String TD3 = "td3";
    public static final String SELF_PLAY = "self_play";

    public static final String MULTI_GRAPH = "multi_graph";
    public static final String TRAIN_MASK = "train_mask";
    public static final String TEST_MASK = "test_mask";
    public static final String GNN = "gnn";
    public static final String NETWORK_TYPE = "network_type";

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

    public static final String ENVIRONMENT_REWARD_TOPIC = "reward";
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

    public static final String ACTOR = "actor";
    public static final String CRITIC = "critic";
    public static final String GENERATOR = "generator";

    public static final String DISCRIMINATOR = "discriminator";
    public static final String QNETWORK = "qnetwork";
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

    public static final String ENCODER = "encoder";
    public static final String DECODER = "decoder";
    public static final String KL_LOSS_WEIGHT = "kl_loss_weight";
    public static final String RECON_LOSS = "reconstruction_loss";
}