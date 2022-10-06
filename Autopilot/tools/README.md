# Plotting

## convert_to_db.py
Using this file, the training log, velocity log, and collision log will be read and converted into two sqlite tables:
**episodes.db**:
 - episode
 - total-reward
 - avg_last_100_reward
 - avg_critic_loss
 - avg_q_vales
 - duration
 - training_steps
 - epsilon
 - number_static_collisions
 - total_time_static_collisions
 - number_vehicle_collisions
 - total_time_vehicle_collisions
**velocities.db**:
 - episode
 - step
 - car_name
 - velocity
 - timestamp

In order to use this script, log collisions and velocities by adding `"collision_mode": "LOG_COLLISIONS"` to the scenario file.

### Usage
```bash
python3 convert_to_db.py -s <snapshot_interval> -e <evaluation_samples> [-a]
```
If using `-a`, the velocity and collision logs will be read from the `~/dev/basic-simulator/` folder. Otherwise, the logs will be read from the `logs` folder of the current experiment.

### Requirements
 - python3
 - sqlite3
 - pandas

## plot.py
Using this file, the sqlite tables will be read and plotted. The plots will be saved in the `logs/figures` folder of the current experiment.

Adjust this file for your needs.

### Requirements
 - python3
 - sqlite3
 - matplotlib