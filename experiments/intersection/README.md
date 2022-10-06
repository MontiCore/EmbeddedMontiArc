# NAME Experiment

## Description
This experiment teaches a small number of vehicles to drive safely across an intersection. For this the intersection randomization is used.

## Goal
The goal is to learn a policy that is able to drive across any kind of intersection with a variable but small number of other vehicles.

## Pass Criteria
The experiment is succesful if the vehicles learn a policy which is capable of driving to their destination without crashing into other vehicles or objects.

## Parameters
### Map
```json
"map_name": "maps/dallas.osm"
```

### Randomization
```json
"randomization": [{
        "type": "intersection",
        "min_distance_from_intersection": 10,
        "max_distance_from_intersection": 10,
        "min_number_of_vehicles": 2,
        "max_number_of_vehicles": 4,
        "min_distance_btw_vehicles": 8,
        "max_distance_btw_vehicles": 8
    }]
```

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
```json
"preprocessor": {
        "type": "sequence",
        "preprocessors": [
            {
                "type": "angle"
            },
            {
                "type": "proximity",
                "maxNumberOfVehicles": 4
            }
        ]
    }
```

### Training
See the src/ folder for more details.

#### Overview
##### State
Size of state for own vehicle: 26
Number of other vehicles: 2 - 4
Size of state for each other vehicle: 26

##### Training
Self-Play: yes
Number of Episodes: 5000
