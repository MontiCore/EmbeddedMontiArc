from re import S
import sqlite3
import os
import matplotlib
import matplotlib.pyplot as plt
from pathlib import Path

AP_TRAINING_AP = Path(os.path.dirname(os.path.abspath(__file__))).parent
PATH_FIGURES = AP_TRAINING_AP / "logs" / "figures"
EPISODES_DATABASE_FILE = AP_TRAINING_AP / "logs" / "episodes.db"
VELOCITIES_DATABASE_FILE = AP_TRAINING_AP / "logs" / "velocities.db"


def plot_total_reward_avg_last_100_reward(con_episodes):
    cur = con_episodes.cursor()
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
    plt.savefig(PATH_FIGURES / "reward.png", dpi=300)
    plt.close()


def plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, x=100):
    cur = con_episodes.cursor()
    cur.execute(
        f"SELECT episode, total_reward, avg_last_100_reward FROM episodes WHERE episode in (SELECT episode FROM episodes ORDER BY episode DESC LIMIT {x}) ORDER BY episode ASC"
    )
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
    plt.savefig(PATH_FIGURES / f"reward_last_{x}.png", dpi=300)
    plt.close()


def plot_avg_last_100_reward(con_episodes):
    cur = con_episodes.cursor()
    cur.execute("SELECT episode, avg_last_100_reward FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_last_100_rewards = [tup[1] for tup in tuples]

    plt.plot(episodes, avg_last_100_rewards, label="Avg. Reward last 100 Episodes")

    plt.xlabel("Episode")
    plt.ylabel("Reward")

    plt.legend()
    plt.savefig(PATH_FIGURES / "avg_100_reward.png", dpi=300)
    plt.close()


def plot_avg_actor_critic_loss(con_episodes):
    cur = con_episodes.cursor()
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
    plt.savefig(PATH_FIGURES / "loss.png", dpi=300)
    plt.close()


def plot_avg_actor_loss(con_episodes):
    cur = con_episodes.cursor()
    cur.execute("SELECT episode, avg_actor_loss FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_actor_losses = [tup[1] for tup in tuples]

    plt.plot(episodes, avg_actor_losses, label="Avg. Actor Loss per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Loss")

    plt.legend()
    plt.savefig(PATH_FIGURES / "actor_loss.png", dpi=300)
    plt.close()


def plot_avg_critic_loss(con_episodes):
    cur = con_episodes.cursor()
    cur.execute("SELECT episode, avg_critic_loss FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_critic_losses = [tup[1] for tup in tuples]

    plt.plot(episodes, avg_critic_losses, label="Avg. Critic Loss per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Loss")

    plt.legend()
    plt.savefig(PATH_FIGURES / "critic_loss.png", dpi=300)
    plt.close()


def plot_avg_q_values(con_episodes):
    cur = con_episodes.cursor()
    cur.execute("SELECT episode, avg_q_values FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    avg_q_values = [tup[1] for tup in tuples]

    plt.plot(episodes, avg_q_values, label="Avg. Q-Values per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Q-Value")

    plt.legend()
    plt.savefig(PATH_FIGURES / "q_value.png", dpi=300)
    plt.close()


def plot_durations_training_steps(con_episodes):
    cur = con_episodes.cursor()
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

    plt.savefig(PATH_FIGURES / "duration_time_steps.png", dpi=300)
    plt.close()

def plot_duration(con_episodes):
    cur = con_episodes.cursor()
    cur.execute("SELECT episode, duration FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    durations = [tup[1] for tup in tuples]

    plt.plot(episodes, durations, label="Duration per Epsiode")

    plt.xlabel("Episode")
    plt.ylabel("Duration (s)")

    plt.legend()

    plt.savefig(PATH_FIGURES / "duration.png", dpi=300)
    plt.close()


def plot_epsilon(con_episodes):
    cur = con_episodes.cursor()
    cur.execute("SELECT episode, epsilon FROM episodes")
    tuples = cur.fetchall()
    tuples = sorted(tuples, key=lambda tup: tup[0])
    episodes = [tup[0] for tup in tuples]
    epsilons = [tup[1] for tup in tuples]

    plt.plot(episodes, epsilons, label="Epsilon per Episode")

    plt.xlabel("Episode")
    plt.ylabel("Epsilon")

    plt.legend()
    plt.savefig(PATH_FIGURES / "epsilon.png", dpi=300)
    plt.close()


def plot_collisions_numbers(con_episodes):
    cur = con_episodes.cursor()
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
    plt.savefig(PATH_FIGURES / "collision_numbers.png", dpi=300)
    plt.close()


def plot_collisions_times(con_episodes):
    cur = con_episodes.cursor()
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
    avg_time_static_collisions = [
        total_time_static_collisions[episode] / number_static_collisions[episode]
        if number_static_collisions[episode] > 0
        else 0
        for episode in episodes
    ]
    avg_time_vehicle_collisions = [
        total_time_vehicle_collisions[episode] / number_vehicle_collisions[episode]
        if number_vehicle_collisions[episode] > 0
        else 0
        for episode in episodes
    ]

    plt.plot(
        episodes, total_time_static_collisions, label="Total Time of Static Collisions"
    )
    plt.plot(
        episodes,
        total_time_vehicle_collisions,
        label="Total Time of Vehicle Collisions",
    )
    plt.plot(
        episodes, avg_time_static_collisions, label="Avg. Time of Static Collisions"
    )
    plt.plot(
        episodes, avg_time_vehicle_collisions, label="Avg. Time of Vehicle Collisions"
    )

    plt.xlabel("Episode")
    plt.ylabel("Seconds")

    plt.legend()
    plt.savefig(PATH_FIGURES / "collision_times.png", dpi=300)
    plt.close()


def plot_avg_collisions_times(con_episodes):
    cur = con_episodes.cursor()
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
    avg_time_static_collisions = [
        total_time_static_collisions[episode] / number_static_collisions[episode]
        if number_static_collisions[episode] > 0
        else 0
        for episode in episodes
    ]
    avg_time_vehicle_collisions = [
        total_time_vehicle_collisions[episode] / number_vehicle_collisions[episode]
        if number_vehicle_collisions[episode] > 0
        else 0
        for episode in episodes
    ]

    plt.plot(
        episodes, avg_time_static_collisions, label="Avg. Time of Static Collisions"
    )
    plt.plot(
        episodes, avg_time_vehicle_collisions, label="Avg. Time of Vehicle Collisions"
    )

    plt.xlabel("Episode")
    plt.ylabel("Seconds")

    plt.legend()
    plt.savefig(PATH_FIGURES / "avg_collision_times.png", dpi=300)
    plt.close()


def plot_total_collisions_times(con_episodes):
    cur = con_episodes.cursor()
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
    avg_time_static_collisions = [
        total_time_static_collisions[episode] / number_static_collisions[episode]
        if number_static_collisions[episode] > 0
        else 0
        for episode in episodes
    ]
    avg_time_vehicle_collisions = [
        total_time_vehicle_collisions[episode] / number_vehicle_collisions[episode]
        if number_vehicle_collisions[episode] > 0
        else 0
        for episode in episodes
    ]

    plt.plot(
        episodes, total_time_static_collisions, label="Total Time of Static Collisions"
    )
    plt.plot(
        episodes,
        total_time_vehicle_collisions,
        label="Total Time of Vehicle Collisions",
    )

    plt.xlabel("Episode")
    plt.ylabel("Seconds")

    plt.legend()
    plt.savefig(PATH_FIGURES / "total_collision_times.png", dpi=300)
    plt.close()


def plot_velocities_interval(con_velocities, interval=50, starting_at=0, reverse=False):
    if starting_at < 1 and not reverse:
        starting_at = 1

    cur = con_velocities.cursor()
    cur.execute(f"SELECT episode, step, car_name, velocity FROM velocities")
    tuples = cur.fetchall()
    max_episode = max([tup[0] for tup in tuples])
    if reverse:
        tuples = [
            tup
            for tup in tuples
            if tup[0] <= max_episode - starting_at
            and tup[0] > max_episode - starting_at - interval
        ]
    else:
        tuples = [
            tup
            for tup in tuples
            if tup[0] >= starting_at and tup[0] < starting_at + interval
        ]

    tuples = sorted(tuples, key=lambda tup: tup[1])

    steps = [tup[1] for tup in tuples]
    steps = sorted(steps)

    # get min velocity for every step
    min_velocities = []
    for step in steps:
        min_velocities.append(min([tup[3] for tup in tuples if tup[1] == step]))

    # get max velocity for every step
    max_velocities = []
    for step in steps:
        max_velocities.append(max([tup[3] for tup in tuples if tup[1] == step]))

    # get avg velocity for every step
    avg_velocities = []
    for step in steps:
        avg_velocities.append(
            sum([tup[3] for tup in tuples if tup[1] == step])
            / len([tup[3] for tup in tuples if tup[1] == step])
        )

    fig, ax = plt.subplots(1)
    ax.plot(
        steps, avg_velocities, label="Average Velocity of all Vehicles", color="blue"
    )
    ax.fill_between(steps, min_velocities, max_velocities, alpha=0.4, color="blue")

    ax.set_xlabel("Step")
    ax.set_ylabel("Velocity")

    plt.legend()
    plt.savefig(
        PATH_FIGURES
        / f"velocities_interval_eps-{starting_at}-{starting_at+interval}_reverse-{reverse}.png",
        dpi=300,
    )
    plt.close()


def plot_velocities_single_episode(con_velocities, episode_at=0, reverse=False):
    if episode_at < 1 and not reverse:
        episode_at = 1

    cur = con_velocities.cursor()
    cur.execute(f"SELECT episode, step, car_name, velocity FROM velocities")
    tuples = cur.fetchall()
    max_episode = max([tup[0] for tup in tuples])
    if reverse:
        tuples = [tup for tup in tuples if tup[0] == max_episode - episode_at]
    else:
        tuples = [tup for tup in tuples if tup[0] == episode_at]

    tuples = sorted(tuples, key=lambda tup: tup[1])

    steps = [tup[1] for tup in tuples]
    steps = sorted(steps)
    steps = list(dict.fromkeys(steps))
    step_count = max(steps)

    vehicle_velocities = dict()
    for tup in tuples:
        if tup[2] not in vehicle_velocities:
            vehicle_velocities[tup[2]] = [None] * (step_count + 1)
        vehicle_velocities[tup[2]][tup[1]] = tup[3]

    for vehicle in vehicle_velocities:
        plt.plot(steps, vehicle_velocities[vehicle], label=vehicle)

    plt.xlabel("Step")
    plt.ylabel("Velocity")

    plt.legend()
    plt.savefig(
        PATH_FIGURES / f"velocities_episode-{episode_at}reverse-{reverse}.png", dpi=300
    )
    plt.close()


con_episodes = sqlite3.connect(EPISODES_DATABASE_FILE)
con_velocities = sqlite3.connect(VELOCITIES_DATABASE_FILE)

plt.rcParams["lines.linewidth"] = 1

plot_total_reward_avg_last_100_reward(con_episodes)
plot_avg_last_100_reward(con_episodes)
plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, 50)
plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, 100)
plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, 200)
plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, 500)
plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, 1000)
plot_last_x_episodes_total_reward_avg_last_100_reward(con_episodes, 10000)
plot_avg_actor_critic_loss(con_episodes)
plot_avg_actor_loss(con_episodes)
plot_avg_critic_loss(con_episodes)
plot_avg_q_values(con_episodes)
plot_durations_training_steps(con_episodes)
plot_duration(con_episodes)
plot_epsilon(con_episodes)
plot_collisions_numbers(con_episodes)
plot_collisions_times(con_episodes)
plot_avg_collisions_times(con_episodes)
plot_total_collisions_times(con_episodes)

for episode in range(1000, 11000, 250):
    plot_velocities_single_episode(con_velocities, episode_at=episode, reverse=False)
    plot_velocities_interval(con_velocities, interval=10, starting_at=episode, reverse=False)


plot_velocities_single_episode(con_velocities, episode_at=0, reverse=True)
plot_velocities_interval(con_velocities, interval=10, starting_at=0, reverse=True)

con_episodes.close()
con_velocities.close()
