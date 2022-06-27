import sqlite3
import re
import os

DATABASE_FILE = os.path.dirname(os.path.abspath(__file__)) + "/../logs/log.db"

INT_REGEX = "[0-9]+"
FLOAT_REGEX = "[-+]?(?:\d*\.\d+|\d+)" #"[-+]?[0-9]*\.?[0-9]+(e[-+]?[0-9]+)"

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
                    epsilon REAL
               );"""
)
con.commit()

logfile = open(os.path.dirname(os.path.abspath(__file__)) + "/../logs/logfile", "r")

for line in logfile.readlines():
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

    cur.execute(
        "INSERT INTO episodes VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
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
        ),
    )

    con.commit()

con.close()
