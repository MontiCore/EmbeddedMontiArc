from reinforcement_learning.agent import DqnAgent
import reinforcement_learning.environment

import CNNCreator_torcs_agent_torcsAgent_dqn
import logging
import mxnet as mx

if __name__ == "__main__":
    env_params = {
        'ros_node_name' : 'torcs_agent_torcsAgent_dqnTrainerNode',
        'state_topic' : 'preprocessor_state',
        'action_topic' : 'postprocessor_action',
        'reset_topic' : 'torcs_reset',
        'terminal_state_topic' : 'prepocessor_is_terminal'
    }
    env = reinforcement_learning.environment.RosEnvironment(**env_params)
    context = mx.cpu()
    net_creator = CNNCreator_torcs_agent_torcsAgent_dqn.CNNCreator_torcs_agent_torcsAgent_dqn()
    net_creator.construct(context)

    replay_memory_params = {
        'method':'buffer',
        'memory_size':1000000,
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
        'epsilon_decay': 0.0001,
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
        training_episodes=20000,
        train_interval=1,
        use_fix_target=True,
        target_update_interval=500,
        double_dqn = True,
        snapshot_interval=1000,
        agent_name='torcs_agent_torcsAgent_dqn',
        max_episode_step=999999999,
        output_directory='model',
        verbose=True,
        live_plot = True,
        make_logfile=True,
        target_score=None
    )
    train_successfull = agent.train()
    agent.save_best_network(net_creator._model_dir_ + net_creator._model_prefix_ + '_newest', epoch=0)