<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
<#setting number_format="computer">
<#assign config = configurations[0]>
<#assign rlAgentType=config.rlAlgorithm?switch("dqn", "DqnAgent", "ddpg", "DdpgAgent", "td3", "TwinDelayedDdpgAgent")>
from ${rlFrameworkModule}.agent import ${rlAgentType}
from ${rlFrameworkModule}.util import AgentSignalHandler
from ${rlFrameworkModule}.cnnarch_logger import ArchLogger
<#if config.rlAlgorithm=="ddpg" || config.rlAlgorithm=="td3">
from ${rlFrameworkModule}.CNNCreator_${criticInstanceName} import CNNCreator_${criticInstanceName}
</#if>
import ${rlFrameworkModule}.environment
import CNNCreator_${config.instanceName}

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
<#if (config.agentName)??>
    agent_name = '${config.agentName}'
<#else>
    agent_name = '${config.instanceName}'
</#if>
    # Prepare output directory and logger
    all_output_dir = os.path.join('model', agent_name)
    output_directory = os.path.join(
        all_output_dir,
        time.strftime('%Y-%m-%d-%H-%M-%S',
                      time.localtime(time.time())))
    ArchLogger.set_output_directory(output_directory)
    ArchLogger.set_logger_name(agent_name)
    ArchLogger.set_output_level(ArchLogger.INFO)

<#if config.environment?? && config.environmentName == "gym">
    env = ${rlFrameworkModule}.environment.GymEnvironment(<#if config.environmentParameters['name']??>'${config.environmentParameters['name']}'<#else>'CartPole-v0'</#if>)
<#else>
    env_params = {
        'ros_node_name': '${config.instanceName}TrainerNode',
<#if config.environment?? && config.environmentParameters['state']??>
        'state_topic': '${config.environmentParameters['state']}',
</#if>
<#if config.environment?? && config.environmentParameters['action']??>
        'action_topic': '${config.environmentParameters['action']}',
</#if>
<#if config.environment?? && config.environmentParameters['reset']??>
        'reset_topic': '${config.environmentParameters['reset']}',
</#if>
<#if config.environment?? && config.environmentParameters['terminal']??>
        'terminal_state_topic': '${config.environmentParameters['terminal']}',
</#if>
<#if config.environment?? && config.environmentParameters['reward']??>
        'reward_topic': '${config.environmentParameters['reward']}',
</#if>
    }
    env = ${rlFrameworkModule}.environment.RosEnvironment(**env_params)
</#if>

<#if (config.context)??>
    context = mx.${config.context}()
<#else>
    context = mx.cpu()
</#if>
<#if (config.initializer)??>
<#if config.initializer.method=="normal">
    initializer_params = {
        'sigma': ${config.initializer.sigma}
    }
    initializer = mx.init.Normal(**initializer_params)
</#if>
<#else>
    initializer = mx.init.Normal()
</#if>
<#if config.rlAlgorithm=="ddpg" || config.rlAlgorithm=="td3">
<#if (config.criticInitializer)??>
<#if config.criticInitializer.method=="normal">
    critic_initializer_params = {
        'sigma': ${config.criticInitializer.sigma}
    }
    critic_initializer = mx.init.Normal(**critic_initializer_params)
</#if>
<#else>
    critic_initializer = mx.init.Normal()
</#if>
</#if>
<#if config.rlAlgorithm == "dqn">
    qnet_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    qnet_creator.setWeightInitializer(initializer)
    qnet_creator.construct([context])
<#elseif config.rlAlgorithm=="ddpg" || config.rlAlgorithm=="td3">
    actor_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    actor_creator.setWeightInitializer(initializer)
    actor_creator.construct([context])
    critic_creator = CNNCreator_${criticInstanceName}()
    critic_creator.setWeightInitializer(critic_initializer)
    critic_creator.construct([context])
</#if>

    agent_params = {
        'environment': env,
        'replay_memory_params': {
<#include "params/ReplayMemoryParams.ftl">
        },
        'strategy_params': {
<#include "params/StrategyParams.ftl">
        },
        'agent_name': agent_name,
        'verbose': True,
        'output_directory': output_directory,
        'state_dim': (<#list config.stateDim as d>${d},</#list>),
        'action_dim': (<#list config.actionDim as d>${d},</#list>),
<#if (config.context)??>
        'ctx': '${config.context}',
</#if>
<#if (config.discountFactor)??>
        'discount_factor': ${config.discountFactor},
</#if>
<#if (config.numEpisodes)??>
        'training_episodes': ${config.numEpisodes},
</#if>
<#if (config.trainingInterval)??>
        'train_interval': ${config.trainingInterval},
</#if>
<#if (config.startTrainingAt)??>
        'start_training': ${config.startTrainingAt},
</#if>
<#if (config.snapshotInterval)??>
        'snapshot_interval': ${config.snapshotInterval},
</#if>
<#if (config.numMaxSteps)??>
        'max_episode_step': ${config.numMaxSteps},
</#if>
<#if (config.evaluationSamples)??>
        'evaluation_samples': ${config.evaluationSamples},
</#if>
<#if (config.selfPlay)??>
        'self_play': '${config.selfPlay}',
</#if>
<#if (config.outputDirectory)??>
        'output_directory': ${config.outputDirectory},
</#if>
<#if (config.targetScore)??>
        'target_score': ${config.targetScore},
</#if>
<#if (config.rlAlgorithm == "dqn")>
<#include "params/DqnAgentParams.ftl">
<#elseif config.rlAlgorithm == "ddpg">
<#include "params/DdpgAgentParams.ftl">
<#else>
<#include "params/Td3AgentParams.ftl">
</#if>
    }

    resume, resume_directory = resume_session(all_output_dir)

    if resume:
        output_directory, _ = os.path.split(resume_directory)
        ArchLogger.set_output_directory(output_directory)
        resume_agent_params = {
            'session_dir': resume_directory,
            'environment': env,
<#if config.rlAlgorithm == "dqn">
            'net': qnet_creator.networks[0],
<#else>
            'actor': actor_creator.networks[0],
            'critic': critic_creator.networks[0]
</#if>
        }
        agent = ${rlAgentType}.resume_from_session(**resume_agent_params)
    else:
        agent = ${rlAgentType}(**agent_params)

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
<#if (config.rlAlgorithm == "dqn")>
        agent.export_best_network(path=str(qnet_creator.get_model_dir(epoch=0) / 'model_0_newest'), epoch=0)
<#else>
        agent.export_best_network(path=str(actor_creator.get_model_dir(epoch=0) / 'model_0_newest'), epoch=0)
</#if>
