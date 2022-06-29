import sqlite3
import os
import matplotlib
import matplotlib.pyplot as plt

PATH = os.path.dirname(os.path.abspath(__file__)) + "/../logs/"
PATH_FIGURES = os.path.dirname(os.path.abspath(__file__)) + "/../logs/figures/"
DATABASE_FILE = PATH + "log.db"


def plot_total_reward_avg_last_100_reward(con):
    cur = con.cursor()
    cur.execute("SELECT episode, total_reward, avg_last_100_reward FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    total_rewards = [tup[1] for tup in tuples]
    avg_last_100_rewards = [tup[2] for tup in tuples]

    plt.plot(episodes, total_rewards, label="Total Reward per Episode")
    plt.plot(episodes, avg_last_100_rewards, label="Avg. Reward last 100 Episodes")

    plt.xlabel("Episode")
    plt.ylabel("Reward")

    plt.legend()
    plt.savefig(PATH_FIGURES + "reward.png")
    plt.close()


def plot_avg_actor_critic_loss(con):
    cur = con.cursor()
    cur.execute("SELECT episode, avg_actor_loss, avg_critic_loss FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_actor_losses = [tup[1] for tup in tuples]
    avg_critic_losses = [tup[2] for tup in tuples]

    plt.plot(episodes, avg_actor_losses, label="Avg. Actor Loss per Episode")
    plt.plot(episodes, avg_critic_losses, label="Avg. Critic Loss per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Loss")

    plt.legend()
    plt.savefig(PATH_FIGURES + "loss.png")
    plt.close()


def plot_avg_q_values(con):
    cur = con.cursor()
    cur.execute("SELECT episode, avg_q_values FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_q_values = [tup[1] for tup in tuples]

    plt.plot(episodes, avg_q_values, label="Avg. Q-Values per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Q-Value")

    plt.legend()
    plt.savefig(PATH_FIGURES + "q_value.png")
    plt.close()


def plot_durations_training_steps(con):
    cur = con.cursor()
    cur.execute("SELECT episode, duration, training_steps FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    durations = [tup[1] for tup in tuples]
    training_steps = [tup[2] for tup in tuples]

    ax1 = plt.subplot()
    ax2 = ax1.twinx()

    plt1 = ax1.plot(episodes, durations, label="Duration per Epsiode")
    plt2 = ax2.plot(episodes, training_steps, label="Time Steps per Epsiode", c="darkorange")

    plts = plt1+plt2
    labels = [p.get_label() for p in plts]
    ax1.legend(plts, labels, loc=4)

    ax1.set_xlabel("Episode")
    ax1.set_ylabel("Duration (s)")
    ax2.set_ylabel("Time Steps")

    plt.savefig(PATH_FIGURES + "duration.png")
    plt.close()


def plot_epsilon(con):
    cur = con.cursor()
    cur.execute("SELECT episode, epsilon FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    epsilons = [tup[1] for tup in tuples]

    plt.plot(episodes, epsilons, label="Epsilon per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Epsilon")

    plt.legend()
    plt.savefig(PATH_FIGURES + "epsilon.png")
    plt.close()


con = sqlite3.connect(DATABASE_FILE)

plot_total_reward_avg_last_100_reward(con)
plot_avg_actor_critic_loss(con)
plot_avg_q_values(con)
plot_durations_training_steps(con)
plot_epsilon(con)

con.close()
