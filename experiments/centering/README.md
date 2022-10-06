# Centering Experiment

## Description
In this experiment two agents will be trained on identical scenarios. One agent will use the CenteringPreprocessor to recenter the environment around a vehicle, while the other agent doesn't.

## Goal
Two vehicles will be trained to drive northwards on similar and straight roads on different sides of the map.

## Research Goal
The goal is to see if the CenteringPreprocessor improves the performance of the agent.

## Pass Criteria
If the rewards stabilize and are converging to some value, the experiment can be terminated.

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
	"standing_punishment": -300,
	"velocity_max": 20,
	"velocity_desired": 15,
	"standing_threshold": 1,
	"standing_punishment_from_step": 5,
}
```

### Preprocessor
None for the first agent. CenteringPreprocessor for the second agent.
```json
{
	"type": "centering"
}
```

### Training
See the src/ folder for more details.

#### Overview
##### State
Size of state for own vehicle: 25
Number of other vehicles: 2
Size of state for each other vehicle: 25

##### Training
Self-Play: Yes
Number of Episodes: 3000
Start training after Episode: 500 (fill replay buffer)
