# (c) https://github.com/MontiCore/monticore
from reinforcement_learning.agent import DdpgAgent
from reinforcement_learning.util import AgentSignalHandler
from reinforcement_learning.cnnarch_logger import ArchLogger
from reinforcement_learning.CNNCreator_RosCriticNetwork import CNNCreator_RosCriticNetwork
import reinforcement_learning.environment
import CNNCreator_rosActorNetwork

import os
import sys
import re
import time
import numpy as np
import mxnet as mx


def resume_session(sessions_dir):
    resume_session = False
    resume_directory = None
    if os.path.isdir(sessions_dir):
        regex = re.compile(r'\d\d\d\d-\d\d-\d\d-\d\d-\d\d')
        dir_content = os.listdir(sessions_dir)
        session_files = list(filter(regex.search, dir_content))
        session_files.sort(reverse=True)
        for d in session_files:
            interrupted_session_dir = os.path.join(sessions_dir, d, '.interrupted_session')
            if os.path.isdir(interrupted_session_dir):
                if sys.version.strip().split(".")[0] > '2':
                    resume = input('Interrupted session from {} found. Do you want to resume? (y/n) '.format(d))
                else:
                    resume = raw_input('Interrupted session from {} found. Do you want to resume? (y/n) '.format(d))
                if resume == 'y':
                    resume_session = True
                    resume_directory = interrupted_session_dir
                break
    return resume_session, resume_directory


if __name__ == "__main__":
    agent_name = 'ddpg-agent'
    # Prepare output directory and logger
    all_output_dir = os.path.join('model', agent_name)
    output_directory = os.path.join(
        all_output_dir,
        time.strftime('%Y-%m-%d-%H-%M-%S',
                      time.localtime(time.time())))
    ArchLogger.set_output_directory(output_directory)
    ArchLogger.set_logger_name(agent_name)
    ArchLogger.set_output_level(ArchLogger.INFO)

    env_params = {
        'ros_node_name': 'rosActorNetworkTrainerNode',
        'state_topic': '/environment/state',
        'action_topic': '/environment/action',
        'reset_topic': '/environment/reset',
        'reward_topic': '/environment/reward',
    }
    env = reinforcement_learning.environment.RosEnvironment(**env_params)

    context = mx.cpu()
    initializer = mx.init.Normal()
    critic_initializer = mx.init.Normal()
    actor_creator = CNNCreator_rosActorNetwork.CNNCreator_rosActorNetwork()
    actor_creator.setWeightInitializer(initializer)
    actor_creator.construct([context])
    critic_creator = CNNCreator_RosCriticNetwork()
    critic_creator.setWeightInitializer(critic_initializer)
    critic_creator.construct([context])

    agent_params = {
        'environment': env,
        'replay_memory_params': {
            'method': 'buffer',
            'memory_size': 1500000,
            'sample_size': 128,
            'state_dtype': 'float32',
            'action_dtype': 'float32',
            'rewards_dtype': 'float32'

        },
        'strategy_params': {
            'method':'ornstein_uhlenbeck',
            'epsilon': 1.0,
            'min_epsilon': 0.001,
            'epsilon_decay_method': 'linear',
            'epsilon_decay': 1.0E-4,
            'epsilon_decay_start': 50,
            'action_low': -1,
            'action_high': 1,
            'mu': [0.0, 0.1, 0.3],
            'theta': [0.5, 0.0, 0.8],
            'sigma': [0.3, 0.6, -0.9],
        },
        'agent_name': agent_name,
        'verbose': True,
        'output_directory': output_directory,
        'state_dim': (8,),
        'action_dim': (3,),
        'discount_factor': 0.9998,
        'training_episodes': 2500,
        'train_interval': 1,
        'start_training': 50,
        'snapshot_interval': 500,
        'max_episode_step': 2000,
        'evaluation_samples': 1000,
        'actor': actor_creator.networks[0],
        'critic': critic_creator.networks[0],
        'soft_target_update_rate': 0.001,
        'actor_optimizer': 'adam',
        'actor_optimizer_params': {
        'learning_rate_minimum': 1.0E-5,
        'learning_rate': 0.001},
        'critic_optimizer': 'rmsprop',
        'critic_optimizer_params': {
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
    }

    resume, resume_directory = resume_session(all_output_dir)

    if resume:
        output_directory, _ = os.path.split(resume_directory)
        ArchLogger.set_output_directory(output_directory)
        resume_agent_params = {
            'session_dir': resume_directory,
            'environment': env,
            'actor': actor_creator.networks[0],
            'critic': critic_creator.networks[0]
        }
        agent = DdpgAgent.resume_from_session(**resume_agent_params)
    else:
        agent = DdpgAgent(**agent_params)

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
        agent.export_best_network(path=str(actor_creator.get_model_dir(epoch=0) / 'model_0_newest'), epoch=0)
