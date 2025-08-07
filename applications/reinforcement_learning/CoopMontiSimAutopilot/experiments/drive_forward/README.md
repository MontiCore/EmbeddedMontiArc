# Drive Forward Experiment

## Description
In this experiment, a single vehicle will be trained to follow a straight line.

## Goal
In this experiment, a single vehicle will be trained to follow a straight line.

## Research Goal
The purpose of this experiment is to verify, that our contributions in the simulator - specifically the reward functions - are working correctly.

## Pass Criteria
If the vehicle tries to follow the path, the experiment is considered a success.

## Parameters
### Map
```json
"map_name": "maps/dallas.osm"
```

### Randomization
**None**
Instead, we use a fixed scenario to ensure a comparable result.
View the `scenario_file.json` for more details.

### Reward
```json
{
	"type": "intersection_reward",
	"static_collision_factor": -400,
	"vehicle_collision_factor": -300,
	"trajectory_reward_factor": 1,
	"angular_jerk_reward_factor": 0.015,
	"angular_acc_reward_factor": 0.015,
	"linear_jerk_reward_factor": 0.0,
	"linear_acc_reward_factor": 0.0,
	"trajectory_following_reward": 30,
	"total_path_progress_reward_scaling": 2000,
	"path_progress_derivative_reward_scaling": 1000,
	"distance_max": 5,
	"velocity_difference_scaling": 100,
	"velocity_sub_maximum_reward": 100,
	"standing_punishment": -300,
	"velocity_max": 20,
	"velocity_desired": 15,
	"standing_threshold": 1,
	"standing_punishment_from_step": 5,
	"standing_punishment_to_min_path_distance": 2
}
```

### Preprocessor
```json
{}
```

### Training
See the src/ folder for more details.

#### Overview
##### State
Size of state for own vehicle: 25
Number of other vehicles: 0
Size of state for each other vehicle: NA

##### Training
Self-Play: No
Number of Episodes: 2000
Start training after Episode: 500 (fill replay buffer)
