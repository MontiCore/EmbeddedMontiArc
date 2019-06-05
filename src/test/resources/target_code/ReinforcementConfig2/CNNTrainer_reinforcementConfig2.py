from reinforcement_learning.agent import DqnAgent
from reinforcement_learning.util import AgentSignalHandler
import reinforcement_learning.environment
import CNNCreator_reinforcementConfig2

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
    env = reinforcement_learning.environment.GymEnvironment('CartPole-v1')
    context = mx.cpu()
    net_creator = CNNCreator_reinforcementConfig2.CNNCreator_reinforcementConfig2()
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
            discount_factor=0.999,
            loss='l2',
            optimizer='rmsprop',
            optimizer_params={
                'weight_decay': 0.01,
                'centered': True,
                'gamma2': 0.9,
                'gamma1': 0.9,
                'clip_weights': 10.0,
                'learning_rate_decay': 0.9,
                'epsilon': 1.0E-6,
                'rescale_grad': 1.1,
                'clip_gradient': 10.0,
                'learning_rate_minimum': 1.0E-5,
                'learning_rate_policy': 'step',
                'learning_rate': 0.001,
                'step_size': 1000},
            training_episodes=200,
            train_interval=1,
            use_fix_target=False,
            double_dqn = False,
            snapshot_interval=20,
            agent_name=agent_name,
            max_episode_step=250,
            output_directory=session_output_dir,
            verbose=True,
            live_plot = True,
            make_logfile=True,
            target_score=185.5
        )

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
        agent.save_best_network(net_creator._model_dir_ + net_creator._model_prefix_ + '_newest', epoch=0)