from reinforcement_learning.agent import DqnAgent
import reinforcement_learning.environment

import CNNCreator_cartpole_master_dqn
import logging
import mxnet as mx

if __name__ == "__main__":
    env = reinforcement_learning.environment.GymEnvironment('CartPole-v0')
    context = mx.cpu()
    net_creator = CNNCreator_cartpole_master_dqn.CNNCreator_cartpole_master_dqn()
    net_creator.construct(context)

    replay_memory_params = {
        'method':'buffer',
        'memory_size':10000,
        'sample_size':32,
        'state_dtype':'float32',
        'action_dtype':'uint8',
        'rewards_dtype':'float32'
    }

    policy_params = {
        'method':'epsgreedy',
        'epsilon': 1,
        'min_epsilon': 0.01,
        'epsilon_decay_method': 'linear',
        'epsilon_decay': 0.01,
    }

    agent = DqnAgent(
        network = net_creator.net,
        environment=env,
        replay_memory_params=replay_memory_params,
        policy_params=policy_params,
        state_dim=net_creator.get_input_shapes()[0],
        ctx='cpu',
        discount_factor=0.999,
        loss_function='euclidean',
        optimizer='rmsprop',
        optimizer_params={
            'learning_rate': 0.001
        },
        training_episodes=160,
        train_interval=1,
        use_fix_target=True,
        target_update_interval=200,
        double_dqn = False,
        snapshot_interval=20,
        agent_name='cartpole_master_dqn',
        max_episode_step=250,
        output_directory='model',
        verbose=True,
        live_plot = True,
        make_logfile=True,
        target_score=185.5
    )
    train_successfull = agent.train()
    agent.save_best_network(net_creator._model_dir_ + net_creator._model_prefix_ + '_newest', epoch=0)