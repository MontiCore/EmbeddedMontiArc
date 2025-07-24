# (c) https://github.com/MontiCore/monticore
import signal
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style
import time
import os
import mxnet
from mxnet import gluon, nd


LOSS_FUNCTIONS = {
        'l1': gluon.loss.L1Loss(),
        'euclidean': gluon.loss.L2Loss(),
        'huber_loss': gluon.loss.HuberLoss(),
        'softmax_cross_entropy': gluon.loss.SoftmaxCrossEntropyLoss(),
        'sigmoid_cross_entropy': gluon.loss.SigmoidBinaryCrossEntropyLoss()}

def copy_net(net, input_state_dim, ctx, tmp_filename='tmp.params'):
    assert isinstance(net, gluon.HybridBlock)
    assert type(net.__class__) is type
    net.save_parameters(tmp_filename)
    net2 = net.__class__()
    net2.load_parameters(tmp_filename, ctx=ctx)
    os.remove(tmp_filename)
    net2.hybridize()
    net2(nd.ones((1,) + input_state_dim, ctx=ctx))
    return net2

def get_loss_function(loss_function_name):
    if loss_function_name not in LOSS_FUNCTIONS:
        raise ValueError('Loss function does not exist')
    return LOSS_FUNCTIONS[loss_function_name]


class AgentSignalHandler(object):
    def __init__(self):
        signal.signal(signal.SIGINT, self.interrupt_training)
        self.__agent = None

    def register_agent(self, agent):
        self.__agent = agent

    def interrupt_training(self, sig, frame):
        if self.__agent:
            self.__agent.set_interrupt_flag(True)

style.use('fivethirtyeight')
class TrainingStats(object):
    def __init__(self, logger, max_episodes, live_plot=True):
        self.__logger = logger
        self.__max_episodes = max_episodes
        self.__all_avg_loss = np.zeros((max_episodes,))
        self.__all_total_rewards = np.zeros((max_episodes,))
        self.__all_eps = np.zeros((max_episodes,))
        self.__all_time = np.zeros((max_episodes,))
        self.__all_mean_reward_last_100_episodes = np.zeros((max_episodes,))
        self.__live_plot = live_plot

    @property
    def logger(self):
        return self.__logger

    @logger.setter
    def logger(self, logger):
        self.__logger = logger

    @logger.deleter
    def logger(self):
        self.__logger = None

    def add_avg_loss(self, episode, avg_loss):
        self.__all_avg_loss[episode] = avg_loss

    def add_total_reward(self, episode, total_reward):
        self.__all_total_rewards[episode] = total_reward

    def add_eps(self, episode, eps):
        self.__all_eps[episode] = eps

    def add_time(self, episode, time):
        self.__all_time[episode] = time

    def add_mean_reward_last_100(self, episode, mean_reward):
        self.__all_mean_reward_last_100_episodes[episode] = mean_reward

    def log_episode(self, episode, start_time, training_steps, loss, eps, reward):
        self.add_eps(episode, eps)
        self.add_total_reward(episode, reward)
        end = time.time()
        if training_steps == 0:
            avg_loss = 0
        else:
            avg_loss = float(loss)/float(training_steps)

        mean_reward_last_100 = self.mean_of_reward(episode, last=100)

        time_elapsed = end - start_time
        info = "Episode: %d, Total Reward: %.3f, Avg. Reward Last 100 Episodes: %.3f, Avg Loss: %.3f, Time: %.3f, Training Steps: %d, Eps: %.3f"\
            % (episode, reward, mean_reward_last_100, avg_loss, time_elapsed, training_steps, eps)
        self.__logger.info(info)
        self.add_avg_loss(episode, avg_loss)
        self.add_time(episode, time_elapsed)
        self.add_mean_reward_last_100(episode, mean_reward_last_100)

        return avg_loss, time_elapsed, mean_reward_last_100

    def mean_of_reward(self, cur_episode, last=100):
        if cur_episode > 0:
            reward_last_100 = self.__all_total_rewards[max(0, cur_episode-last):cur_episode]
            return np.mean(reward_last_100)
        else:
            return self.__all_total_rewards[0]

    def save_stats(self, file):
        fig = plt.figure(figsize=(20,20))

        sub_rewards = fig.add_subplot(221)
        sub_rewards.set_title('Total Rewards per episode')
        sub_rewards.plot(np.arange(self.__max_episodes), self.__all_total_rewards)

        sub_loss = fig.add_subplot(222)
        sub_loss.set_title('Avg. Loss per episode')
        sub_loss.plot(np.arange(self.__max_episodes), self.__all_avg_loss)

        sub_eps = fig.add_subplot(223)
        sub_eps.set_title('Epsilon per episode')
        sub_eps.plot(np.arange(self.__max_episodes), self.__all_eps)

        sub_rewards = fig.add_subplot(224)
        sub_rewards.set_title('Avg. mean reward of last 100 episodes')
        sub_rewards.plot(np.arange(self.__max_episodes), self.__all_mean_reward_last_100_episodes)

        plt.savefig(file)
