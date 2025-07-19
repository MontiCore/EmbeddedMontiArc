from curses.ascii import SI
import sqlite3
import re
import os
from pathlib import Path
import pandas as pd

BASIC_SIMULATOR_FOLDER = Path("~/dev/basic-simulator/").expanduser()
SIMULATOR_RESULTS = Path(BASIC_SIMULATOR_FOLDER / "install" / "results").glob(
    "collision_log_training_*.csv"
)
LATEST_SIMULATOR_RESULT = max(SIMULATOR_RESULTS, key=os.path.getctime)

AP_TRAINING_AP = Path(os.path.dirname(os.path.abspath(__file__))).parent
DATABASE_FILE = AP_TRAINING_AP / "logs" / "log.db"
LOG_FILE = AP_TRAINING_AP / "logs" / "logfile"
df = pd.read_csv(LATEST_SIMULATOR_RESULT)

INT_REGEX = "[0-9]+"
FLOAT_REGEX = "[-+]?(?:\d*\.\d+|\d+)"  # "[-+]?[0-9]*\.?[0-9]+(e[-+]?[0-9]+)"

if os.path.exists(DATABASE_FILE):
    os.remove(DATABASE_FILE)

open(DATABASE_FILE, "a").close()

con = sqlite3.connect(DATABASE_FILE)

cur = con.cursor()

cur.execute(
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
con.commit()

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

    for _, row in df.loc[
        (df["Episode"] == int(episode)) & (df["CollisionType"] == "static")
    ].iterrows():
        number_static_collisions += 1
        total_time_static_collisions += row["Duration"]

    for _, row in df.loc[
        (df["Episode"] == int(episode)) & (df["CollisionType"] == "vehicle")
    ].iterrows():
        number_vehicle_collisions += 1
        total_time_vehicle_collisions += row["Duration"]

    cur.execute(
        "INSERT INTO episodes VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
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
        ),
    )

    con.commit()

con.close()
