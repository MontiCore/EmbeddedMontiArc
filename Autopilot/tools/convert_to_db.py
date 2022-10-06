import sqlite3
import re
import os
from pathlib import Path
import pandas as pd
import sys, getopt
import math


AP_TRAINING_AP = Path(os.path.dirname(os.path.abspath(__file__))).parent
BASIC_SIMULATOR_FOLDER = Path("~/dev/basic-simulator/").expanduser()
OTHER_LOG_DIRECTORY = Path(AP_TRAINING_AP / "logs").expanduser()

argv = sys.argv[1:]
try:
    opts, args = getopt.getopt(
        argv, "has:e:", ["automatic", "snapshot_interval=", "evaluation_samples="]
    )
except getopt.GetoptError:
    print("python convert_to_db.py -s <snapshot_interval> -e <evaluation_samples> [-a]")
    sys.exit(2)
for opt, arg in opts:
    if opt == "-h":
        print("python convert_to_db.py -s <snapshot_interval> -e <evaluation_samples> [-a]")
        sys.exit()
    if opt == "-a":
        OTHER_LOG_DIRECTORY = Path(BASIC_SIMULATOR_FOLDER / "install" / "results")
    elif opt in ("-s", "--snapshot_interval"):
        snapshot_interval = arg
    elif opt in ("-e", "--evaluation_samples"):
        evaluation_samples = arg

COLLISION_LOGS = Path(OTHER_LOG_DIRECTORY).glob("collision_log_training_*.csv")
LATEST_COLLISION_LOG = max(COLLISION_LOGS, key=os.path.getctime)
VELOCITY_LOGS = Path(OTHER_LOG_DIRECTORY).glob("velocity_log_training_*.csv")
LATEST_VELOCITY_LOG = max(VELOCITY_LOGS, key=os.path.getctime)

EPISODES_DATABASE_FILE = AP_TRAINING_AP / "logs" / "episodes.db"
VELOCITIES_DATABASE_FILE = AP_TRAINING_AP / "logs" / "velocities.db"
LOG_FILE = AP_TRAINING_AP / "logs" / "logfile"
df_collisons = pd.read_csv(LATEST_COLLISION_LOG)
df_velocities = pd.read_csv(LATEST_VELOCITY_LOG)

INT_REGEX = "[0-9]+"
FLOAT_REGEX = "[-+]?(?:\d*\.\d+|\d+)"  # "[-+]?[0-9]*\.?[0-9]+(e[-+]?[0-9]+)"

if os.path.exists(EPISODES_DATABASE_FILE):
    os.remove(EPISODES_DATABASE_FILE)
open(EPISODES_DATABASE_FILE, "a").close()
con_episodes = sqlite3.connect(EPISODES_DATABASE_FILE)
cur_episodes = con_episodes.cursor()
cur_episodes.execute(
    """CREATE TABLE episodes
               (
                    episode INTEGER PRIMARY KEY,
                    total_reward REAL,
                    avg_last_100_reward REAL,
                    avg_actor_loss REAL,
                    avg_critic_loss REAL,
                    avg_q_values REAL,
                    duration REAL,
                    training_steps INTEGER,
                    epsilon REAL,
                    number_static_collisions INTEGER,
                    total_time_static_collisions REAL,
                    number_vehicle_collisions INTEGER,
                    total_time_vehicle_collisions REAL
               );"""
)
con_episodes.commit()

if os.path.exists(VELOCITIES_DATABASE_FILE):
    os.remove(VELOCITIES_DATABASE_FILE)
open(VELOCITIES_DATABASE_FILE, "a").close()
con_velocities = sqlite3.connect(VELOCITIES_DATABASE_FILE)
cur_velocities = con_velocities.cursor()
cur_velocities.execute(
    """CREATE TABLE velocities
               (
                    episode INTEGER,
                    step INTEGER,
                    car_name TEXT,
                    velocity REAL,
                    timestamp REAL
               );"""
)
con_velocities.commit()


episode_rows = list()
velocity_rows = list()
for line in open(LOG_FILE, "r").readlines():
    match = re.search(
        f"^(?P<start_time>.*) - (?P<agent_name>.*) - INFO - Episode: (?P<episode>{INT_REGEX}), Total Reward: (?P<total_reward>{FLOAT_REGEX}), Avg. Reward Last 100 Episodes: (?P<avg_last_100_reward>{FLOAT_REGEX}), Avg. Actor Loss: (?P<avg_actor_loss>{FLOAT_REGEX}) Avg. Critic Loss: (?P<avg_critic_loss>{FLOAT_REGEX}) Avg. Q-Values: (?P<avg_q_values>{FLOAT_REGEX}), Time: (?P<duration>{FLOAT_REGEX}), Training Steps: (?P<training_steps>{INT_REGEX}), Eps: (?P<epsilon>{FLOAT_REGEX})",
        line,
    )

    if match is None:
        continue

    episode = match.group("episode")
    total_reward = match.group("total_reward")
    avg_last_100_reward = match.group("avg_last_100_reward")
    avg_actor_loss = match.group("avg_actor_loss")
    avg_critic_loss = match.group("avg_critic_loss")
    avg_q_values = match.group("avg_q_values")
    duration = match.group("duration")
    training_steps = match.group("training_steps")
    epsilon = match.group("epsilon")

    number_static_collisions = 0
    total_time_static_collisions = 0
    number_vehicle_collisions = 0
    total_time_vehicle_collisions = 0

    for _, row in df_collisons.loc[
        (
            df_collisons["Episode"]
            == int(episode)
            + math.floor(int(episode) / int(snapshot_interval))
            * int(evaluation_samples)
        )
        & (df_collisons["CollisionType"] == "static")
    ].iterrows():
        number_static_collisions += 1
        total_time_static_collisions += row["Duration"]

    for _, row in df_collisons.loc[
        (
            df_collisons["Episode"]
            == int(episode)
            + math.floor(int(episode) / int(snapshot_interval))
            * int(evaluation_samples)
        )
        & (df_collisons["CollisionType"] == "vehicle")
    ].iterrows():
        number_vehicle_collisions += 1
        total_time_vehicle_collisions += row["Duration"]

    for _, row in df_velocities.loc[
        (
            df_velocities["Episode"]
            == int(episode)
            + math.floor(int(episode) / int(snapshot_interval))
            * int(evaluation_samples)
        )
    ].iterrows():
        velocity_rows.append(
            (
                episode,
                row["Step"],
                row["VehicleName"],
                row["Velocity"],
                row["TimeStamp"],
            )
        )

    episode_rows.append(
        (
            episode,
            total_reward,
            avg_last_100_reward,
            avg_actor_loss,
            avg_critic_loss,
            avg_q_values,
            duration,
            training_steps,
            epsilon,
            number_static_collisions,
            total_time_static_collisions,
            number_vehicle_collisions,
            total_time_vehicle_collisions,
        )
    )


cur_episodes.executemany(
    "INSERT INTO episodes VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", episode_rows
)
con_episodes.commit()

cur_velocities.executemany(
    "INSERT INTO velocities VALUES (?, ?, ?, ?, ?)", velocity_rows
)
con_velocities.commit()

con_episodes.close()
con_velocities.close()
