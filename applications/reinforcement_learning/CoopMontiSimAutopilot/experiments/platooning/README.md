# Vehicle Platooning Experiment

## Description
In this experiment an agent will be trained to perform vehicle platooning.

## Goal
Each vehicle has to follow its path. If a vehicle shares a path with other vehicles, they should minimize the gap between them (platooning).

## Research Goal
Train an agent to perform platooning using the newly developed randomization, reward, and communication framework.

## Pass Criteria
If the rewards stabilize and are converging to some value, the experiment can be terminated.

## Parameters
### Map
```json
"map_name": "maps/dallas.osm"
```

### Randomization
```json
[
	{
		"type": "platooning",
		"minNumberOfVehicles": 2,
		"maxNumberOfVehicles": 5,
		"minDistanceBtwVehicles": 10,
		"maxDistanceBtwVehicles": 20,
		"minGoalDistance": 20,
		"maxGoalDistance": 300
	}
]
```

### Reward
```json
{
	"type": "platooning_reward",
	"GAP_DISTANCE_REWARD_SCALING": 10,
	"GAP_SUB_MAXIMUM_REWARD": 100,
	"gap_max": 20,
	"gap_desired": 10,
	"speed_control_reward_properties": {
		"type": "speed_control_reward",
		"VELOCITY_DIFFERENCE_REWARD_SCALING": 100,
		"VELOCITY_SUB_MAXIMUM_REWARD": 100,
		"STANDING_REWARD": -250,
		"LINEAR_ACCELERATION_REWARD_SCALING": 1,
		"LINEAR_JERK_REWARD_SCALING": 0.05,
		"ANGULAR_ACCELERATION_REWARD_SCALING": 0.1,
		"ANGULAR_JERK_REWARD_SCALING": 0.005,
		"velocity_max": 20,
		"velocity_desired": 15,
		"standing_threshold": 1,
		"standing_punishment_from_step": 5,
		"standing_punishment_to_min_path_distance": 2
	},
	"static_collision_reward_properties": {
		"type": "static_collision_reward",
		"STATIC_COLLISION_REWARD": -400
	},
	"vehicle_collision_reward_properties": {
		"type": "vehicle_collision_reward",
		"VEHICLE_COLLISIONS_REWARD": -300
	},
	"trajectory_reward_properties": {
		"type": "trajectory_reward",
		"TRAJECTORY_FOLLOWING_REWARD": 50,
		"TOTAL_PATH_PROGRESS_REWARD_SCALING": 2000,
		"PATH_PROGRESS_DERIVATIVE_REWARD_SCALING": 1000,
		"distance_max": 3
	}
}
```

### Preprocessor
```json
{
	"type": "sequence",
	"preprocessors": [
		{
			"type": "trajectory",
			"maxNumberOfVehicles": 2,
			"MAX_STATES_PER_VEHICLE": 4,
			"addIndicator": true
		},
		{
			"type": "angle"
		}
	]
}
```
The results from the centering experiment have shown, that the CenteringPreprocessor should not be used. However, the AnglePreprocessor helps with training as shown in the angle representation experiment.

### Training
See the src/ folder for more details.

#### Overview
##### State
Size of state for own vehicle: 26
Number of other vehicles: 2 (using TrajectoryFilter)
Size of state for each other vehicle: 27

##### Training
Self-Play: Yes
Number of Episodes: 11000
Start training after Episode: 1000 (fill replay buffer)
