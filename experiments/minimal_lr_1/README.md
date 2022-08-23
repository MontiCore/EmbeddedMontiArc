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

### Randomization
```json
{
	"type": "basic",
	"minGoalDistance": 500,
	"maxGoalDistance": 1000
}
```

### Reward
```json
{
	"type": "basic_reward",
	"static_collision_reward_properties": {
		"STATIC_COLLISION_REWARD": -500
	},
	"trajectory_reward_properties": {
		"TRAJECTORY_FOLLOWING_REWARD": 1,
		"TOTAL_PATH_PROGRESS_REWARD_SCALING": 10,
		"PATH_PROGRESS_DERIVATIVE_REWARD_SCALING": 100,
		"distance_max": 5
	},
	"speed_control_reward_properties": {
		"VELOCITY_DIFFERENCE_REWARD_SCALING": 1,
		"VELOCITY_SUB_MAXIMUM_REWARD": 0.5,
		"STANDING_REWARD": -10,
		"LINEAR_ACCELERATION_REWARD_SCALING": 1,
		"LINEAR_JERK_REWARD_SCALING": 0.05,
		"ANGULAR_ACCELERATION_REWARD_SCALING": 0.1,
		"ANGULAR_JERK_REWARD_SCALING": 0.005,
		"velocity_max": 40,
		"velocity_desired": 20,
		"standing_threshold": 0.5,
		"standing_punishment_from_step": 5,
		"standing_punishment_to_min_path_distance": 2.5
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
Number of other vehicles: 0
Size of state for each other vehicle: NA

##### Training
Self-Play: No
Number of Episodes: 500
