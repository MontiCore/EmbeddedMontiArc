from reinforcement_learning.agent import TwinDelayedDdpgAgent
from reinforcement_learning.util import AgentSignalHandler
from reinforcement_learning.cnnarch_logger import ArchLogger
from reinforcement_learning.CNNCreator_cheetah_agent_cheetahCritic import CNNCreator_cheetah_agent_cheetahCritic
import reinforcement_learning.environment
import CNNCreator_cheetah_master_cheetah

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
        session_files = filter(regex.search, dir_content)
        session_files.sort(reverse=True)
        for d in session_files:
            interrupted_session_dir = os.path.join(sessions_dir, d, '.interrupted_session')
            if os.path.isdir(interrupted_session_dir):
                resume = raw_input('Interrupted session from {} found. Do you want to resume? (y/n) '.format(d))
                if resume == 'y':
                    resume_session = True
                    resume_directory = interrupted_session_dir
                break
    return resume_session, resume_directory


if __name__ == "__main__":
    agent_name = 'CheetahAgent'
    # Prepare output directory and logger
    all_output_dir = os.path.join('model', agent_name)
    output_directory = os.path.join(
        all_output_dir,
        time.strftime('%Y-%m-%d-%H-%M-%S',
                      time.localtime(time.time())))
    ArchLogger.set_output_directory(output_directory)
    ArchLogger.set_logger_name(agent_name)
    ArchLogger.set_output_level(ArchLogger.INFO)

    env = reinforcement_learning.environment.GymEnvironment('RoboschoolHalfCheetah-v1')

    context = mx.gpu()
    actor_creator = CNNCreator_cheetah_master_cheetah.CNNCreator_cheetah_master_cheetah()
    actor_creator.construct(context)
    critic_creator = CNNCreator_cheetah_agent_cheetahCritic()
    critic_creator.construct(context)

    agent_params = {
        'environment': env,
        'replay_memory_params': {
            'method': 'buffer',
            'memory_size': 1000000,
            'sample_size': 100,
            'state_dtype': 'float32',
            'action_dtype': 'float32',
            'rewards_dtype': 'float32'
        },
        'strategy_params': {
            'method':'gaussian',
            'epsilon': 1,
            'min_epsilon': 0.05,
            'epsilon_decay_method': 'linear',
            'epsilon_decay': 0.005,
            'epsilon_decay_start': 1000,
            'noise_variance': 0.1,
            'action_low': -1,
            'action_high': 1,
        },
        'agent_name': agent_name,
        'verbose': True,
        'output_directory': output_directory,
        'state_dim': (26,),
        'action_dim': (6,),
        'ctx': 'gpu',
        'discount_factor': 0.99,
        'training_episodes': 2500,
        'train_interval': 1,
        'start_training': 1,
        'snapshot_interval': 250,
        'max_episode_step': 10000,
        'evaluation_samples': 100,
        'actor': actor_creator.networks[0],
        'critic': critic_creator.networks[0],
        'soft_target_update_rate': 0.005,
        'actor_optimizer': 'adam',
        'actor_optimizer_params': {
            'learning_rate': 0.001},
        'critic_optimizer': 'adam',
        'critic_optimizer_params': {
            'learning_rate': 0.001},
        'policy_noise': 0.2,
        'noise_clip': 0.5,
        'policy_delay': 2,
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
        agent = TwinDelayedDdpgAgent.resume_from_session(**resume_agent_params)
    else:
        agent = TwinDelayedDdpgAgent(**agent_params)

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
        agent.export_best_network(path=actor_creator._model_dir_ + actor_creator._model_prefix_ + '_0_newest', epoch=0)
