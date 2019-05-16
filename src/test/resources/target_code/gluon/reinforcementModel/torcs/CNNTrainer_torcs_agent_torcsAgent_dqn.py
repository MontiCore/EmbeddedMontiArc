from reinforcement_learning.agent import DqnAgent
from reinforcement_learning.util import AgentSignalHandler
import reinforcement_learning.environment
import CNNCreator_torcs_agent_torcsAgent_dqn

import os
import sys
import re
import logging
import mxnet as mx

session_output_dir = 'session'
agent_name='torcs_agent_torcsAgent_dqn'
session_param_output = os.path.join(session_output_dir, agent_name)

def resume_session():
    session_param_output = os.path.join(session_output_dir, agent_name)
    resume_session = False
    resume_directory = None
    if os.path.isdir(session_output_dir) and os.path.isdir(session_param_output):
        regex = re.compile(r'\d\d\d\d-\d\d-\d\d-\d\d-\d\d')
        dir_content = os.listdir(session_param_output)
        session_files = filter(regex.search, dir_content)
        session_files.sort(reverse=True)
        for d in session_files:
            interrupted_session_dir = os.path.join(session_param_output, d, '.interrupted_session')
            if os.path.isdir(interrupted_session_dir):
                resume = raw_input('Interrupted session from {} found. Do you want to resume? (y/n) '.format(d))
                if resume == 'y':
                    resume_session = True
                    resume_directory = interrupted_session_dir
                break
    return resume_session, resume_directory

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

    resume_session, resume_directory = resume_session()

    if resume_session:
        agent = DqnAgent.resume_from_session(resume_directory, net_creator.net, env)
    else:
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
                'learning_rate': 0.001            },
            training_episodes=20000,
            train_interval=1,
            use_fix_target=True,
            target_update_interval=500,
            double_dqn = True,
            snapshot_interval=1000,
            agent_name=agent_name,
            max_episode_step=999999999,
            output_directory=session_output_dir,
            verbose=True,
            live_plot = True,
            make_logfile=True,
            target_score=None
        )

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
        agent.save_best_network(net_creator._model_dir_ + net_creator._model_prefix_ + '_newest', epoch=0)