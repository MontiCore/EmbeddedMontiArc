<#setting number_format="computer">
<#assign config = configurations[0]>
from ${rlFrameworkModule}.agent import DqnAgent
from ${rlFrameworkModule}.util import AgentSignalHandler
import ${rlFrameworkModule}.environment
import CNNCreator_${config.instanceName}

import os
import sys
import re
import logging
import mxnet as mx

session_output_dir = 'session'
<#if (config.agentName)??>
agent_name='${config.agentName}'
<#else>
agent_name='${config.instanceName}'
</#if>
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
<#if config.environment.environment == "gym">
    env = ${rlFrameworkModule}.environment.GymEnvironment(<#if config.environment.name??>'${config.environment.name}'<#else>'CartPole-v0'</#if>)
<#else>
    env_params = {
        'ros_node_name' : '${config.instanceName}TrainerNode',
<#if config.environment.state_topic??>
        'state_topic' : '${config.environment.state_topic}',
</#if>
<#if config.environment.action_topic??>
        'action_topic' : '${config.environment.action_topic}',
</#if>
<#if config.environment.reset_topic??>
        'reset_topic' : '${config.environment.reset_topic}',
</#if>
<#if config.environment.meta_topic??>
        'meta_topic' : '${config.environment.meta_topic}',
</#if>
<#if config.environment.greeting_topic??>
        'greeting_topic' : '${config.environment.greeting_topic}'
</#if>
<#if config.environment.terminal_state_topic??>
        'terminal_state_topic' : '${config.environment.terminal_state_topic}'
</#if>
    }
    env = ${rlFrameworkModule}.environment.RosEnvironment(**env_params)
</#if>
<#if (config.context)??>
    context = mx.${config.context}()
<#else>
    context = mx.cpu()
</#if>
    net_creator = CNNCreator_${config.instanceName}.CNNCreator_${config.instanceName}()
    net_creator.construct(context)

    replay_memory_params = {
<#if (config.replayMemory)??>
        'method':'${config.replayMemory.method}',
<#if (config.replayMemory.memory_size)??>
        'memory_size':${config.replayMemory.memory_size},
</#if>
<#if (config.replayMemory.sample_size)??>
        'sample_size':${config.replayMemory.sample_size},
</#if>
<#else>
        'method':'online',
</#if>
        'state_dtype':'float32',
        'action_dtype':'uint8',
        'rewards_dtype':'float32'
    }

    policy_params = {
<#if (config.actionSelection)??>
        'method':'${config.actionSelection.method}',
<#else>
        'method':'epsgreedy'
</#if>
<#if (config.actionSelection.epsilon)??>
        'epsilon': ${config.actionSelection.epsilon},
</#if>
<#if (config.actionSelection.min_epsilon)??>
        'min_epsilon': ${config.actionSelection.min_epsilon},
</#if>
<#if (config.actionSelection.epsilon_decay_method)??>
        'epsilon_decay_method': '${config.actionSelection.epsilon_decay_method}',
</#if>
<#if (config.actionSelection.epsilon_decay)??>
        'epsilon_decay': ${config.actionSelection.epsilon_decay},
</#if>
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
<#if (config.context)??>
            ctx='${config.context}',
</#if>
<#if (config.discountFactor)??>
            discount_factor=${config.discountFactor},
</#if>
<#if (config.loss)??>
            loss_function='${config.loss}',
</#if>
<#if (config.configuration.optimizer)??>
            optimizer='${config.optimizerName}',
            optimizer_params={
<#list config.optimizerParams?keys as param>
                '${param}': ${config.optimizerParams[param]}<#sep>,
</#list>
            },
</#if>
<#if (config.numEpisodes)??>
            training_episodes=${config.numEpisodes},
</#if>
<#if (config.trainingInterval)??>
            train_interval=${config.trainingInterval},
</#if>
<#if (config.useFixTargetNetwork)?? && config.useFixTargetNetwork>
            use_fix_target=True,
            target_update_interval=${config.targetNetworkUpdateInterval},
<#else>
            use_fix_target=False,
</#if>
<#if (config.useDoubleDqn)?? && config.useDoubleDqn>
            double_dqn = True,
<#else>
            double_dqn = False,
</#if>
<#if (config.snapshotInterval)??>
            snapshot_interval=${config.snapshotInterval},
</#if>
            agent_name=agent_name,
<#if (config.numMaxSteps)??>
            max_episode_step=${config.numMaxSteps},
</#if>
            output_directory=session_output_dir,
            verbose=True,
            live_plot = True,
            make_logfile=True,
<#if (config.targetScore)??>
            target_score=${config.targetScore}
<#else>
            target_score=None
</#if>
        )

    signal_handler = AgentSignalHandler()
    signal_handler.register_agent(agent)

    train_successful = agent.train()

    if train_successful:
        agent.save_best_network(net_creator._model_dir_ + net_creator._model_prefix_ + '_newest', epoch=0)