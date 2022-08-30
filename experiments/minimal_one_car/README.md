# Minimal Driving Experiment

## Description
This experiment will be used as a point of reference to compare the value of different preprocessors in a basic driving scenario.

In this experiment, a single vehicle will learn to follow a path, without the use of any preprocessor.

# Goal
The training goal of the autopilot is to follow a given path as closely and as fast as possible, without causing any collisions.

## Research Goal
This experiment is expected to perform worse than the comparative experiments using preprocessors. So the goal of this experiment is to get some result after training until the loss-functions stabilize.

## Pass Criteria
Given the research goal, this experiment is considered passed, if the vehicle will drive in some fashion after the loss-function stabilizes.

## Parameters
### Map
```json
"map_name": "maps/dallas.osm"
```

### Randomization
**None**
Instead, we use a fixed scenario to ensure a comparable result.

### Reward
```json
{
		"type": "intersection_reward",
		"static_collision_factor": -800,
		"vehicle_collision_factor": -600,
		"trajectory_reward_factor": 1,
		"angular_jerk_reward_factor": 0.005,
		"angular_acc_reward_factor": 0.1,
		"linear_jerk_reward_factor": 0.05,
		"linear_acc_reward_factor": 1,
		"trajectory_following_reward": 5,
		"total_path_progress_reward_scaling": 10,
		"path_progress_derivative_reward_scaling": 100,
		"distance_max": 5,
		"velocity_difference_scaling": 1,
		"velocity_sub_maximum_reward": 0.5,
		"standing_punishment": -250,
		"velocity_max": 40,
		"velocity_desired": 20,
		"standing_threshold": 5,
		"standing_punishment_from_step": 15,
		"standing_punishment_to_min_path_distance": 5
	}
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
Number of other vehicles: 1
Size of state for each other vehicle: 25

##### Training
Self-Play: Yes
Number of Episodes: 400
