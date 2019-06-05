from reinforcement_learning.agent import DqnAgent
from reinforcement_learning.util import AgentSignalHandler
import reinforcement_learning.environment
import CNNCreator_reinforcementConfig1

import os
import sys
import re
import logging
import mxnet as mx

session_output_dir = 'session'
agent_name='reinforcement_agent'
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
        'ros_node_name' : 'reinforcementConfig1TrainerNode',
        'state_topic' : '/environment/state',
        'action_topic' : '/environment/action',
        'reset_topic' : '/environment/reset',
    }
    env = reinforcement_learning.environment.RosEnvironment(**env_params)
    context = mx.cpu()
    net_creator = CNNCreator_reinforcementConfig1.CNNCreator_reinforcementConfig1()
    net_creator.construct(context)

    replay_memory_params = {
        'method':'buffer',
        'memory_size':1000000,
        'sample_size':64,
        'state_dtype':'float32',
        'action_dtype':'uint8',
        'rewards_dtype':'float32'
    }

    policy_params = {
        'method':'epsgreedy',
        'epsilon': 1,
        'min_epsilon': 0.02,
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
            discount_factor=0.99999,
            loss='huber',
            loss_params={
                'rho': 0.9},
            optimizer='adam',
            optimizer_params={
                'learning_rate': 0.001},
            training_episodes=1000,
            train_interval=1,
            use_fix_target=True,
            target_update_interval=500,
            double_dqn = True,
            snapshot_interval=500,
            agent_name=agent_name,
            max_episode_step=10000,
            output_directory=session_output_dir,
            verbose=True,
            live_plot = True,
            make_logfile=True,
            target_score=35000
        )

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
        agent.save_best_network(net_creator._model_dir_ + net_creator._model_prefix_ + '_newest', epoch=0)