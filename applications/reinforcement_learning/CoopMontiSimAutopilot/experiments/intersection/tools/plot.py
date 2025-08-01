import sqlite3
import os
import matplotlib
import matplotlib.pyplot as plt
from pathlib import Path

AP_TRAINING_AP = Path(os.path.dirname(os.path.abspath(__file__))).parent
PATH_FIGURES = AP_TRAINING_AP / "logs" / "figures"
DATABASE_FILE = AP_TRAINING_AP / "logs" / "log.db"


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
    plt.savefig(PATH_FIGURES / "reward.png")
    plt.close()


def plot_last_x_episodes_total_reward_avg_last_100_reward(con, x=100):
    cur = con.cursor()
    cur.execute(f"SELECT episode, total_reward, avg_last_100_reward FROM episodes WHERE episode in (SELECT episode FROM episodes ORDER BY episode DESC LIMIT {x}) ORDER BY episode ASC")
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
    plt.savefig(PATH_FIGURES / f"reward_last_{x}.png")
    plt.close()


def plot_avg_last_100_reward(con):
    cur = con.cursor()
    cur.execute("SELECT episode, avg_last_100_reward FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_last_100_rewards = [tup[1] for tup in tuples]

    plt.plot(episodes, avg_last_100_rewards, label="Avg. Reward last 100 Episodes")

    plt.xlabel("Episode")
    plt.ylabel("Reward")

    plt.legend()
    plt.savefig(PATH_FIGURES / "avg_100_reward.png")
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
    plt.savefig(PATH_FIGURES / "loss.png")
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
    plt.savefig(PATH_FIGURES / "q_value.png")
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
    plt2 = ax2.plot(
        episodes, training_steps, label="Time Steps per Epsiode", c="darkorange"
    )

    plts = plt1 + plt2
    labels = [p.get_label() for p in plts]
    ax1.legend(plts, labels, loc=4)

    ax1.set_xlabel("Episode")
    ax1.set_ylabel("Duration (s)")
    ax2.set_ylabel("Time Steps")

    plt.savefig(PATH_FIGURES / "duration.png")
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
    plt.savefig(PATH_FIGURES / "epsilon.png")
    plt.close()


def plot_collisions_numbers(con):
    cur = con.cursor()
    cur.execute(
        "SELECT episode, number_static_collisions, number_vehicle_collisions FROM episodes"
    )
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    number_static_collisions = [tup[1] for tup in tuples]
    number_vehicle_collisions = [tup[2] for tup in tuples]

    plt.plot(episodes, number_static_collisions, label="Number of Static Collisions")
    plt.plot(episodes, number_vehicle_collisions, label="Number of Vehicle Collisions")

    plt.xlabel("Episode")
    plt.ylabel("Number of Collisions")

    plt.legend()
    plt.savefig(PATH_FIGURES / "collision_numbers.png")
    plt.close()


def plot_collisions_times(con):
    cur = con.cursor()
    cur.execute(
        "SELECT episode, number_static_collisions, total_time_static_collisions, number_vehicle_collisions, total_time_vehicle_collisions FROM episodes"
    )
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    number_static_collisions = [tup[1] for tup in tuples]
    total_time_static_collisions = [tup[2] for tup in tuples]
    number_vehicle_collisions = [tup[3] for tup in tuples]
    total_time_vehicle_collisions = [tup[4] for tup in tuples]
    avg_time_static_collisions = [total_time_static_collisions[episode] / number_static_collisions[episode] if number_static_collisions[episode] > 0 else 0 for episode in episodes]
    avg_time_vehicle_collisions = [total_time_vehicle_collisions[episode] / number_vehicle_collisions[episode] if number_vehicle_collisions[episode] > 0 else 0 for episode in episodes]

    plt.plot(episodes, total_time_static_collisions, label="Total Time of Static Collisions")
    plt.plot(episodes, total_time_vehicle_collisions, label="Total Time of Vehicle Collisions")
    plt.plot(episodes, avg_time_static_collisions, label="Avg. Time of Static Collisions")
    plt.plot(episodes, avg_time_vehicle_collisions, label="Avg. Time of Vehicle Collisions")

    plt.xlabel("Episode")
    plt.ylabel("Seconds")

    plt.legend()
    plt.savefig(PATH_FIGURES / "collision_times.png")
    plt.close()



con = sqlite3.connect(DATABASE_FILE)

plot_total_reward_avg_last_100_reward(con)
plot_avg_last_100_reward(con)
plot_last_x_episodes_total_reward_avg_last_100_reward(con, 50)
plot_last_x_episodes_total_reward_avg_last_100_reward(con, 100)
plot_last_x_episodes_total_reward_avg_last_100_reward(con, 200)
plot_last_x_episodes_total_reward_avg_last_100_reward(con, 500)
plot_last_x_episodes_total_reward_avg_last_100_reward(con, 1000)
plot_avg_actor_critic_loss(con)
plot_avg_q_values(con)
plot_durations_training_steps(con)
plot_epsilon(con)
plot_collisions_numbers(con)
plot_collisions_times(con)

con.close()