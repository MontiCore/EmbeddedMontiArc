# NAME Basic

## Description

This experiment is used to test whether the RL toolchain works.
By executing the install_and_run_training.sh script the agent is build and then started.
The training only runs for two episodes and thus doesn't learn anything.

The paths to the simulator and to the generator can be configured in the config.sh file.

## Goal

Run a training session successfuly.

## Pass Criteria

During this test neither the simulator nor the agent crashes.

## Parameters
### Map
```json
"map_name": "maps/dallas.osm"
```

### Randomization

none

### Reward
```json
"rewardFunction": {
		"type": "intersection_reward",
		"static_collision_factor": -400,
		"vehicle_collision_factor": -300,
		"trajectory_reward_factor": 1,
		"angular_jerk_reward_factor": 0.005,
		"angular_acc_reward_factor": 0.1,
		"linear_jerk_reward_factor": 0.05,
		"linear_acc_reward_factor": 1,
		"trajectory_following_reward": 50,
		"total_path_progress_reward_scaling": 2000,
		"path_progress_derivative_reward_scaling": 1000,
		"distance_max": 3,
		"velocity_difference_scaling": 100,
		"velocity_sub_maximum_reward": 100,
		"standing_punishment": -250,
		"velocity_max": 20,
		"velocity_desired": 15,
		"standing_threshold": 1,
		"standing_punishment_from_step": 5,
		"standing_punishment_to_min_path_distance": 2
	}
```

### Preprocessor

none

### Training
See the src/ folder for more details.

#### Overview
##### State
Size of state for own vehicle: 25
Number of other vehicles: 0
Size of state for each other vehicle: 0

##### Training
Self-Play: no
Number of Episodes: 2
