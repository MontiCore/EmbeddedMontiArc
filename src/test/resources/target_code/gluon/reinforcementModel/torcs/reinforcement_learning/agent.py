import mxnet as mx
import numpy as np
import time
import os
import logging
import sys
import util
import matplotlib.pyplot as plt
from replay_memory import ReplayMemoryBuilder
from action_policy import ActionPolicyBuilder
from util import copy_net, get_loss_function
from mxnet import nd, gluon, autograd

class DqnAgent(object):
    def __init__(self,
        network,
        environment,
        replay_memory_params,
        policy_params,
        state_dim,
        ctx=None,
        discount_factor=.9,
        loss_function='euclidean',
        optimizer='rmsprop',
        optimizer_params = {'learning_rate':0.09},
        training_episodes=50,
        train_interval=1,
        use_fix_target=False,
        double_dqn = False,
        target_update_interval=10,
        snapshot_interval=200,
        agent_name='Dqn_agent',
        max_episode_step=99999,
        output_directory='model_parameters',
        verbose=True,
        live_plot = True,
        make_logfile=True,
        target_score=None):
        assert 0 < discount_factor <= 1
        assert train_interval > 0
        assert target_update_interval > 0
        assert snapshot_interval > 0
        assert max_episode_step > 0
        assert training_episodes > 0
        assert replay_memory_params is not None
        assert type(state_dim) is tuple

        self.__ctx = mx.gpu() if ctx == 'gpu' else mx.cpu()
        self.__qnet = network

        self.__environment = environment
        self.__discount_factor = discount_factor
        self.__training_episodes = training_episodes
        self.__train_interval = train_interval
        self.__verbose = verbose
        self.__state_dim = state_dim
        self.__action_dim = self.__qnet(nd.random_normal(shape=((1,) + self.__state_dim), ctx=self.__ctx)).shape[1:]

        replay_memory_params['state_dim'] = state_dim
        self.__replay_memory_params = replay_memory_params
        rm_builder = ReplayMemoryBuilder()
        self.__memory = rm_builder.build_by_params(**replay_memory_params)
        self.__minibatch_size = self.__memory.sample_size

        policy_params['action_dim'] = self.__action_dim
        self.__policy_params = policy_params
        p_builder = ActionPolicyBuilder()
        self.__policy = p_builder.build_by_params(**policy_params)

        self.__target_update_interval = target_update_interval
        self.__target_qnet = copy_net(self.__qnet, self.__state_dim, ctx=self.__ctx)
        self.__loss_function_str = loss_function
        self.__loss_function = get_loss_function(loss_function)
        self.__agent_name = agent_name
        self.__snapshot_interval = snapshot_interval
        self.__creation_time = time.time()
        self.__max_episode_step = max_episode_step
        self.__optimizer = optimizer
        self.__optimizer_params = optimizer_params
        self.__make_logfile = make_logfile
        self.__double_dqn = double_dqn
        self.__use_fix_target = use_fix_target
        self.__live_plot = live_plot
        self.__user_given_directory = output_directory
        self.__target_score = target_score

        self.__interrupt_flag = False

        # Training Context
        self.__current_episode = 0
        self.__total_steps = 0

        # Initialize best network
        self.__best_net = copy_net(self.__qnet, self.__state_dim, self.__ctx)
        self.__best_avg_score = None

        # Gluon Trainer definition
        self.__training_stats = None

        # Prepare output directory and logger
        self.__output_directory = output_directory\
            + '/' + self.__agent_name\
            + '/' + time.strftime('%d-%m-%Y-%H-%M-%S', time.localtime(self.__creation_time))
        self.__logger = self.__setup_logging()
        self.__logger.info('Agent created with following parameters: {}'.format(self.__make_config_dict()))

    @classmethod
    def from_config_file(cls, network, environment, config_file_path, ctx=None):
        import json
        # Load config
        with open(config_file_path, 'r') as config_file:
            config_dict = json.load(config_file)
        return cls(network, environment, ctx=ctx, **config_dict)

    @classmethod
    def resume_from_session(cls, session_dir, network_type):
        import pickle
        session_dir = os.path.join(session_dir, '.interrupted_session')
        if not os.path.exists(session_dir):
            raise ValueError('Session directory does not exist')

        files = dict()
        files['agent'] = os.path.join(session_dir, 'agent.p')
        files['best_net_params'] = os.path.join(session_dir, 'best_net.params')
        files['q_net_params'] = os.path.join(session_dir, 'qnet.params')
        files['target_net_params'] = os.path.join(session_dir, 'target_net.params')

        for file in files.values():
            if not os.path.exists(file):
                raise ValueError('Session directory is not complete: {} is missing'.format(file))

        with open(files['agent'], 'rb') as f:
            agent = pickle.load(f)

        agent.__qnet = network_type()
        agent.__qnet.load_parameters(files['q_net_params'], agent.__ctx)
        agent.__qnet.hybridize()
        agent.__qnet(nd.ones((1,) + agent.__environment.state_dim))
        agent.__best_net = network_type()
        agent.__best_net.load_parameters(files['best_net_params'], agent.__ctx)
        agent.__target_qnet = network_type()
        agent.__target_qnet.load_parameters(files['target_net_params'], agent.__ctx)

        agent.__logger = agent.__setup_logging(append=True)
        agent.__training_stats.logger = agent.__logger
        agent.__logger.info('Agent was retrieved; Training can be continued')

        return agent

    def __interrupt_training(self):
        import pickle
        self.__logger.info('Training interrupted; Store state for resuming')
        session_dir = os.path.join(self.__output_directory, '.interrupted_session')
        if not os.path.exists(session_dir):
            os.mkdir(session_dir)

        del self.__training_stats.logger
        logger = self.__logger
        self.__logger = None

        self.__save_net(self.__qnet, 'qnet', session_dir)
        self.__qnet = None
        self.__save_net(self.__best_net, 'best_net', session_dir)
        self.__best_net = None
        self.__save_net(self.__target_qnet, 'target_net', session_dir)
        self.__target_qnet = None

        agent_session_file = os.path.join(session_dir, 'agent.p')

        with open(agent_session_file, 'wb') as f:
            pickle.dump(self, f)

        logger.info('State successfully stored')

    @property
    def current_episode(self):
        return self.__current_episode

    @property
    def environment(self):
        return self.__environment

    def __adjust_optimizer_params(self, optimizer_params):
        if 'weight_decay' in optimizer_params:
            optimizer_params['wd'] = optimizer_params['weight_decay']
            del optimizer_params['weight_decay']
        if 'learning_rate_decay' in optimizer_params:
            min_learning_rate = 1e-8
            if 'learning_rate_minimum' in optimizer_params:
                min_learning_rate = optimizer_params['learning_rate_minimum']
                del optimizer_params['learning_rate_minimum']
            optimizer_params['lr_scheduler'] = mx.lr.scheduler.FactorScheduler(
                optimizer_params['step_size'],
                factor=optimizer_params['learning_rate_decay'],
                stop_factor_lr=min_learning_rate)
            del optimizer_params['step_size']
            del optimizer_params['learning_rate_decay']

        return optimizer_params

    def set_interrupt_flag(self, interrupt):
        self.__interrupt_flag = interrupt


    def __make_output_directory_if_not_exist(self):
        assert self.__output_directory
        if not os.path.exists(self.__output_directory):
            os.makedirs(self.__output_directory)

    def __setup_logging(self, append=False):
        assert self.__output_directory
        assert self.__agent_name

        output_level = logging.DEBUG if self.__verbose else logging.WARNING
        filemode = 'a' if append else 'w'

        logformat = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        dateformat = '%d-%b-%y %H:%M:%S'
        formatter = logging.Formatter(fmt=logformat, datefmt=dateformat)

        logger = logging.getLogger('DQNAgent')
        logger.setLevel(output_level)

        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setLevel(output_level)
        stream_handler.setFormatter(formatter)
        logger.addHandler(stream_handler)

        if self.__make_logfile:
            self.__make_output_directory_if_not_exist()
            log_file = os.path.join(self.__output_directory, self.__agent_name + '.log')
            file_handler = logging.FileHandler(log_file, mode=filemode)
            file_handler.setLevel(output_level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

        return logger

    def __is_target_reached(self, avg_reward):
        return self.__target_score is not None\
            and avg_reward > self.__target_score


    def get_q_values(self, state, with_best=False):
        return self.get_batch_q_values(nd.array([state], ctx=self.__ctx), with_best=with_best)[0]

    def get_batch_q_values(self, state_batch, with_best=False):
        return self.__best_net(state_batch) if with_best else self.__qnet(state_batch)

    def get_next_action(self, state, with_best=False):
        q_values = self.get_q_values(state, with_best=with_best)
        action = q_values.asnumpy().argmax()
        return q_values.asnumpy().argmax()

    def __sample_from_memory(self):
        states, actions, rewards, next_states, terminals\
            = self.__memory.sample(batch_size=self.__minibatch_size)
        states = nd.array(states, ctx=self.__ctx)
        actions = nd.array(actions, ctx=self.__ctx)
        rewards = nd.array(rewards, ctx=self.__ctx)
        next_states = nd.array(next_states, ctx=self.__ctx)
        terminals = nd.array(terminals, ctx=self.__ctx)
        return states, actions, rewards, next_states, terminals

    def __determine_target_q_values(self, states, actions, rewards, next_states, terminals):
        if self.__use_fix_target:
            q_max_val = self.__target_qnet(next_states)
        else:
            q_max_val = self.__qnet(next_states)

        if self.__double_dqn:
            q_values_next_states = self.__qnet(next_states)
            target_rewards = rewards + nd.choose_element_0index(q_max_val, nd.argmax_channel(q_values_next_states))\
                * (1.0 - terminals) * self.__discount_factor
        else:
            target_rewards = rewards + nd.choose_element_0index(q_max_val, nd.argmax_channel(q_max_val))\
                * (1.0 - terminals) * self.__discount_factor

        target_qval = self.__qnet(states)
        for t in range(target_rewards.shape[0]):
            target_qval[t][actions[t]] = target_rewards[t]

        return target_qval

    def __train_q_net_step(self, trainer):
        states, actions, rewards, next_states, terminals = self.__sample_from_memory()
        target_qval = self.__determine_target_q_values(states, actions, rewards, next_states, terminals)
        with autograd.record():
            q_values = self.__qnet(states)
            loss = self.__loss_function(q_values, target_qval)
        loss.backward()
        trainer.step(self.__minibatch_size)
        return loss

    def __do_snapshot_if_in_interval(self, episode):
        do_snapshot = (episode % self.__snapshot_interval == 0)
        if do_snapshot:
            self.save_parameters(episode=episode)
            self.__evaluate()

    def __do_target_update_if_in_interval(self, total_steps):
        do_target_update = (self.__use_fix_target and total_steps % self.__target_update_interval == 0)
        if do_target_update:
            self.__logger.info('Target network is updated after {} steps'.format(total_steps))
            self.__target_qnet = copy_net(self.__qnet, self.__state_dim, self.__ctx)

    def train(self, episodes=None):
        self.__logger.info("--- Start training ---")
        trainer = gluon.Trainer(self.__qnet.collect_params(), self.__optimizer, self.__adjust_optimizer_params(self.__optimizer_params))
        episodes = episodes if episodes != None else self.__training_episodes

        resume = (self.__current_episode > 0)
        if resume:
            self.__logger.info("Training session resumed")
            self.__logger.info("Starting from episode {}".format(self.__current_episode))
        else:
            self.__training_stats = util.TrainingStats(self.__logger, episodes, self.__live_plot)

        # Implementation Deep Q Learning described by Mnih et. al. in Playing Atari with Deep Reinforcement Learning
        while self.__current_episode < episodes:
            if self.__interrupt_flag:
                self.__interrupt_flag = False
                self.__interrupt_training()
                return False

            step = 0
            episode_reward = 0
            start = time.time()
            state = self.__environment.reset()
            episode_loss = 0
            training_steps = 0
            while step < self.__max_episode_step:
                #1. Choose an action based on current game state and policy
                q_values = self.__qnet(nd.array([state], ctx=self.__ctx))
                action = self.__policy.select_action(q_values[0])

                #2. Play the game for a single step
                next_state, reward, terminal, _ = self.__environment.step(action)

                #3. Store transition in replay memory
                self.__memory.append(state, action, reward, next_state, terminal)

                #4. Train the network if in interval
                do_training = (self.__total_steps % self.__train_interval == 0\
                    and self.__memory.is_sample_possible(self.__minibatch_size))
                if do_training:
                    loss = self.__train_q_net_step(trainer)
                    loss_sum = sum(loss).asnumpy()[0]
                    episode_loss += float(loss_sum)/float(self.__minibatch_size)
                    training_steps += 1

                # Update target network if in interval
                self.__do_target_update_if_in_interval(self.__total_steps)

                step += 1
                self.__total_steps += 1
                episode_reward += reward
                state = next_state

                if terminal:
                    episode_loss = episode_loss if training_steps > 0 else None
                    _, _, avg_reward = self.__training_stats.log_episode(self.__current_episode, start, training_steps,
                        episode_loss, self.__policy.cur_eps, episode_reward)
                    break

            self.__do_snapshot_if_in_interval(self.__current_episode)
            self.__policy.decay()

            if self.__is_target_reached(avg_reward):
                self.__logger.info('Target score is reached in average; Training is stopped')
                break

            self.__current_episode += 1

        self.__evaluate()
        training_stats_file = os.path.join(self.__output_directory, 'training_stats.pdf')
        self.__training_stats.save_stats(training_stats_file)
        self.__logger.info('--------- Training finished ---------')
        return True

    def __save_net(self, net, filename, filedir=None):
        filedir = self.__output_directory if filedir is None else filedir
        filename = os.path.join(filedir, filename + '.params')
        net.save_parameters(filename)


    def save_parameters(self, episode=None, filename='dqn-agent-params'):
        assert self.__output_directory
        self.__make_output_directory_if_not_exist()

        if(episode != None):
            self.__logger.info('Saving model parameters after episode %d' % episode)
            filename = filename + '-ep{}'.format(episode)
        else:
            self.__logger.info('Saving model parameters')
        self.__save_net(self.__qnet, filename)

    def evaluate(self, target=None, sample_games=100, verbose=True):
        target = self.__target_score if target is None else target
        if target:
            target_achieved = 0
        total_reward = 0

        for g in range(sample_games):
            state = self.__environment.reset()
            step = 0
            game_reward = 0
            while step < self.__max_episode_step:
                action = self.get_next_action(state)
                state, reward, terminal, _ = self.__environment.step(action)
                game_reward += reward

                if terminal:
                    if verbose:
                        info = 'Game %d: Reward %f' % (g,game_reward)
                        self.__logger.debug(info)
                    if target:
                        if game_reward >= target:
                            target_achieved += 1
                    total_reward += game_reward
                    break

                step += 1

        avg_reward = float(total_reward)/float(sample_games)
        info = 'Avg. Reward: %f' % avg_reward
        if target:
            target_achieved_ratio = int((float(target_achieved)/float(sample_games))*100)
            info += '; Target Achieved in %d%% of games' % (target_achieved_ratio)

        if verbose:
            self.__logger.info(info)
        return avg_reward

    def __evaluate(self, verbose=True):
        sample_games = 100
        avg_reward = self.evaluate(sample_games=sample_games, verbose=False)
        info = 'Evaluation -> Average Reward in {} games: {}'.format(sample_games, avg_reward)

        if self.__best_avg_score is None or self.__best_avg_score <= avg_reward:
            self.__best_net = copy_net(self.__qnet, self.__state_dim, self.__ctx)
            self.__best_avg_score = avg_reward
            info += ' (NEW BEST)'

        if verbose:
            self.__logger.info(info)



    def play(self, update_frame=1, with_best=False):
        step = 0
        state = self.__environment.reset()
        total_reward = 0
        while step < self.__max_episode_step:
            action = self.get_next_action(state, with_best=with_best)
            state, reward, terminal, _ = self.__environment.step(action)
            total_reward += reward
            do_update_frame = (step % update_frame == 0)
            if do_update_frame:
                self.__environment.render()
                time.sleep(.100)

            if terminal:
                break

            step += 1
        return total_reward

    def save_best_network(self, path, epoch=0):
        self.__logger.info('Saving best network with average reward of {}'.format(self.__best_avg_score))
        self.__best_net.export(path, epoch=epoch)

    def __make_config_dict(self):
        config = dict()
        config['discount_factor'] = self.__discount_factor
        config['optimizer'] = self.__optimizer
        config['optimizer_params'] = self.__optimizer_params
        config['policy_params'] = self.__policy_params
        config['replay_memory_params'] = self.__replay_memory_params
        config['loss_function'] = self.__loss_function_str
        config['optimizer'] = self.__optimizer
        config['training_episodes'] = self.__training_episodes
        config['train_interval'] = self.__train_interval
        config['use_fix_target'] = self.__use_fix_target
        config['double_dqn'] = self.__double_dqn
        config['target_update_interval'] = self.__target_update_interval
        config['snapshot_interval']= self.__snapshot_interval
        config['agent_name'] = self.__agent_name
        config['max_episode_step'] = self.__max_episode_step
        config['output_directory'] = self.__user_given_directory
        config['verbose'] = self.__verbose
        config['live_plot'] = self.__live_plot
        config['make_logfile'] = self.__make_logfile
        config['target_score'] = self.__target_score
        return config

    def save_config_file(self):
        import json
        self.__make_output_directory_if_not_exist()
        filename = os.path.join(self.__output_directory, 'config.json')
        config = self.__make_config_dict()
        with open(filename, mode='w') as fp:
            json.dump(config, fp, indent=4)