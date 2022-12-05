/* (c) https://github.com/MontiCore/monticore */
import Environment;
import ReplayMemory;

schema Reinforcement extends General {

    rl_algorithm = dqn: schema {
        dqn, ddpg, td3;
    }

    self_play: enum {
	no, yes;
    }

    agent_name: string
    num_episodes = 50: N1
    num_max_steps = 99999: N
    discount_factor = 0.99: Q(0:1]
    target_score: Q
    training_interval = 1: N1
    start_training_at: N
    evaluation_samples: N
    snapshot_interval: N1

    actor_optimizer: optimizer_type
    environment_gym: environment_type!
    replay_memory = buffer: replay_memory_type
}
