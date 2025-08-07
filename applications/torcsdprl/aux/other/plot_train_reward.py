import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import re
import os
import time

LOGPATH = "../../logs/"
SAVEPATH = "../../"
NAMES = ["TorcsAgent-affordance"]

BATCH_START = 0
LIMIT = 125000

VALIDATION_SIZE = 100

X_AXIS_BATCH = True # use itterations instead if false
Y_SCALE= "linear"
SMOOTHING = True

SAVE = True


def get_reward(filename):
    rows = open(LOGPATH + filename + ".log").read().strip()
    episodes = set(re.findall(r'Episode: \d+', rows))
    episodes = np.arange(len(episodes))
    reward = []
    smoothed_reward = []

    for e in episodes:
        step = re.findall(r'(Episode: ' + str(e) + ', Total Reward: )(-*\d+.\d+)', rows)
        step_avg = re.findall(r'(Episode: ' + str(e) + ', Total Reward: -*\d+.\d+, Avg. Reward Last 100 Episodes: )(-*\d*.\d*)', rows)
        cur_reward = float(step[0][1])
        cur_smoothed_reward = float(step_avg[0][1])
        reward.append(cur_reward)
        smoothed_reward.append(cur_smoothed_reward)
    return reward, smoothed_reward

def plot_reward(rewards, smoothed_rewards, labels):
    if SAVE:
        matplotlib.use("pgf")
        matplotlib.rcParams.update({
            "pgf.texsystem": "pdflatex",
            'font.family': 'serif',
            'text.usetex': True,
            'pgf.rcfonts': False,
        })
     # plt.style.use("ggplot")
    fig = plt.figure()
    ax = fig.add_subplot(2, 1, 1)
    ax.set_yscale(Y_SCALE)
    plt.ylabel="Total Reward"
    plt.xlabel="Episode"
    for reward, smoothed_reward, label in zip(rewards, smoothed_rewards, labels):
        plt.plot(reward[ :LIMIT], label=label + "Total Reward")
        if SMOOTHING:
            plt.plot(smoothed_reward[ :LIMIT], label=label + "Avg. 100 Eps.")
    if SMOOTHING:
        plt.legend()

    if SAVE:
        plt.savefig(SAVEPATH + "reward")
    else:
        plt.show()


def getFilename(name):
    files = os.listdir("../logs")
    tmp = []
    for file_name in files:
        if len(re.findall(r'' + name, file_name)) > 0:
            tmp.append(file_name)
        
    


if __name__ == "__main__":
    labels = [""]
    reward_array = []
    smoothed_reward_array = []
    for name in NAMES:
        reward, smoothed_reward = get_reward(name)
        reward_array.append(reward)
        smoothed_reward_array.append(smoothed_reward)
        print("Trained for %i episodes" %(len(reward)))
    if SMOOTHING:
        plot_reward(reward_array, smoothed_reward_array, labels)
    else:
        plot_reward(reward_array, reward_array, labels)

        


