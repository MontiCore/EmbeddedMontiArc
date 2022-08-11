# Reward Functions

The reward functions are used to train reinforcement learning agents.
There are several reward functions for different purposes each, that can be modified using properties, and combined using the SumRewardFunction.

## Usage
In the [basic simulator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator) the reward functions can be specified inside the scenario files as an object:
```json
{
  ...,
  "rewardFunction" : {
    "type": "vehicle_collision_reward",
    "VEHICLE_COLLISIONS_REWARD": -800 // <-- Properties are optional
  },
  ...
}
```

## Reward Functions:
### Vehicle Collisions
```json
"type": "vehicle_collision_reward"
```
This reward function punishes vehicle collisions.

For each pair of colliding vehicles 'VEHICLE_COLLISIONS_REWARD' is rewarded.

#### Properties
| Property                  | Type    | Default Value | Short Description                             | Usage                               |
|---------------------------|---------|---------------|-----------------------------------------------|-------------------------------------|
| VEHICLE_COLLISIONS_REWARD | `float` | -800          | Punishment for colliding with another vehicle | `"VEHICLE_COLLISIONS_REWARD": -400` |


### Static Collisions
```json
"type": "static_collision_reward"
```
This reward function punishes static collisions, such as collisions of the vehicle with walls.

For each static collision 'VEHICLE_COLLISIONS_REWARD' is rewarded.

#### Properties
| Property                | Type    | Default Value | Short Description                             | Usage                               |
|-------------------------|---------|---------------|-----------------------------------------------|-------------------------------------|
| STATIC_COLLISION_REWARD | `float` | -600          | Punishment for colliding with a static object | `"VEHICLE_COLLISIONS_REWARD": -900` |


### Speed Control
```json
"type": "speed_control_reward"
```
This reward function rewards speed control. That includes the following:
- Reward depending on how close the vehicle's velocity is to the desired velocity
- Reward if the vehicle's velocity is less than the maximum allowed velocity
- Punishment if the vehicle is standing still after standing_punishment_from_step simulation steps and if not close to its goal
- Reward depending on the control effort (acceleration)
- Reward depending on the derivative of the control effort

#### Properties
| Property                                 | Type    | Default Value | Short Description                                                                     | Usage                                           |
|------------------------------------------|---------|---------------|---------------------------------------------------------------------------------------|-------------------------------------------------|
| VELOCITY_DIFFERENCE_REWARD_SCALING       | `float` | 1             | Scaling of the reward given to the difference in vehicle velocity to desired velocity | `"VELOCITY_DIFFERENCE_REWARD_SCALING": 2`       |
| VELOCITY_SUB_MAXIMUM_REWARD              | `float` | 0.5           | Reward given to a vehicle driving slower than the maximum speed                       | `"VELOCITY_SUB_MAXIMUM_REWARD": 20`             |
| STANDING_REWARD                          | `float` | -10           | Punishment given to a vehicle standing still                                          | `"STANDING_REWARD": -80`                        |
| LINEAR_ACCELERATION_REWARD_SCALING       | `float` | 1             | Scaling of the reward given to the smoothness of the acceleration                     | `"LINEAR_ACCELERATION_REWARD_SCALING": 1`       |
| LINEAR_JERK_REWARD_SCALING               | `float` | 0.05          | Scaling of the reward given to the smoothness of the change in acceleration           | `"LINEAR_JERK_REWARD_SCALING": 0.05`            |
| ANGULAR_ACCELERATION_REWARD_SCALING      | `float` | 0             | How much angular acceleration should be punished                                      | `"ANGULAR_ACCELERATION_REWARD_SCALING": 0`      |
| ANGULAR_JERK_REWARD_SCALING              | `float` | 0             | How much angular jerk should be punished                                              | `"ANGULAR_JERK_REWARD_SCALING": 0`              |
| velocity_max                             | `float` | 40            | Maximum allowed velocity                                                              | `"velocity_max": 20`                            |
| velocity_desired                         | `float` | 20            | Desired velocity                                                                      | `"velocity_desired": 10`                        |
| standing_threshold                       | `float` | 0.5           | Maximum velocity of a vehicle considered to be standing still                         | `"standing_threshold": 1`                       |
| standing_punishment_from_step            | `int`   | 5             | Allows punishment of standing still only after some simulation steps                  | `"standing_punishment_from_step": 10`           |
| standing_punishment_to_min_path_distance | `float` | 2.5           | Disallows punishment of standing still when close to the vehicle's goal               | `"standing_punishment_to_min_path_distance": 8` |


### Trajectory
```json
"type": "trajectory_reward"
```
This reward function rewards a vehicle following its path. That includes the following:
- Rewards the distance of the vehicle to its trajectory
- Rewards the total progress the vehicle has made on its path
- Rewards the change in progress of the vehicle on its path

#### Properties
| Property                                | Type    | Default Value | Short Description                                                                  | Usage                                           |
|-----------------------------------------|---------|---------------|------------------------------------------------------------------------------------|-------------------------------------------------|
| TRAJECTORY_FOLLOWING_REWARD             | `float` | 1             | Scaling of the reward given to how close the vehicle is to the trajectory          | `"TRAJECTORY_FOLLOWING_REWARD": 2`              |
| TOTAL_PATH_PROGRESS_REWARD_SCALING      | `float` | 10            | Scaling of the reward given to the total progress the vehicle has made on the path | `"TOTAL_PATH_PROGRESS_REWARD_SCALING": 20`      |
| PATH_PROGRESS_DERIVATIVE_REWARD_SCALING | `float` | 100           | Scaling of the reward given to the change in progress on the path                  | `"PATH_PROGRESS_DERIVATIVE_REWARD_SCALING": 10` |
| distance_max                            | `float` | 5             | Maximum allowed distance from the vehicle to its trajectory                        | `"distance_max": 10`                            |

### Derivative

This reward function punishes high linear and angular derivatives and jerks.

#### Properties
| Property                         | Type    | Default Value | Short Description                                  | Usage                              |
|----------------------------------|---------|---------------|----------------------------------------------------|------------------------------------|
| angular_jerk_reward_factor       | `float` | 10            | How much angular jerk should be punished           | `"angular_jerk_reward_factor": 10` |
| angular_acc_reward_factor        | `float` | 10            | How much angular acceleration should be punished   | `"angular_acc_reward_factor": 10`  |
| linear_jerk_reward_factor        | `float` | 10            | How much linear jerk should be punished            | `"linear_jerk_reward_factor": 10`  |
| linear_acc_reward_factor         | `float` | 10            | How much linear acceleration should be punished    | `"linear_acc_reward_factor": 10`   |
| quadratic                        | `bool`  | `false`       | Whether the different components should be squared | `"quadratic": false`               |


### Basic Driving
```json
"type": "basic_reward"
```
This reward function was made to be used in combination with the Basic Randomization Strategy. It is composed of the following reward functions:
- Static Collisions
- Vehicle Collisions
- Speed Control
- Trajectory

Here, you can **overwrite** the default properties of these reward functions.

#### Properties
| Property                            | Type     | Default Value                         | Short Description                                    | Usage                                                     |
|-------------------------------------|----------|---------------------------------------|------------------------------------------------------|-----------------------------------------------------------|
| static_collision_reward_properties  | `object` | `{"STATIC_COLLISION_REWARD": -500}`   | Properties of the Static Collisions Reward Function  | `"static_collision_reward_properties": {/*properties*/}`  |
| trajectory_reward_properties        | `object` | `{"distance_max": 5}`                 | Properties of the Trajectory Reward Function         | `"trajectory_reward_properties": {/*properties*/}`        |
| speed_control_reward_properties     | `object` | `{}`                                  | Properties of the Speed Control Reward Function      | `"speed_control_reward_properties": {/*properties*/}`     |


### Platooning
```json
"type": "platooning_reward"
```
This reward function was made to be used in combination with the Platooning Randomization Strategy. It is composed of the following reward functions:
- Static Collisions
- Vehicle Collisions
- Speed Control
- Trajectory

Here, you can **overwrite** the default properties of these reward functions.

In addition, it also rewards the distance of a vehicle to its preceding vehicle in the platoon, if it has any.

#### Properties
| Property                            | Type     | Default Value                         | Short Description                                                            | Usage                                                     |
|-------------------------------------|----------|---------------------------------------|------------------------------------------------------------------------------|-----------------------------------------------------------|
| static_collision_reward_properties  | `object` | `{"STATIC_COLLISION_REWARD": -500}`   | Properties of the Static Collisions Reward Function                          | `"static_collision_reward_properties": {/*properties*/}`  |
| vehicle_collision_reward_properties | `object` | `{"VEHICLE_COLLISIONS_REWARD": -500}` | Properties of the Vehicle Collisions Reward Function                         | `"vehicle_collision_reward_properties": {/*properties*/}` |
| trajectory_reward_properties        | `object` | `{"distance_max": 5}`                 | Properties of the Trajectory Reward Function                                 | `"trajectory_reward_properties": {/*properties*/}`        |
| speed_control_reward_properties     | `object` | `{}`                                  | Properties of the Speed Control Reward Function                              | `"speed_control_reward_properties": {/*properties*/}`     |
| GAP_DISTANCE_REWARD_SCALING         | `float`  | 20                                    | Scaling of the reward depending on the distance to the preceding vehicle     | `"GAP_DISTANCE_REWARD_SCALING": 10`                       |
| GAP_SUB_MAXIMUM_REWARD              | `float`  | 1                                     | Reward if the distance to the preceding vehicle is less than the maximum gap | `"GAP_SUB_MAXIMUM_REWARD": 5`                             |
| gap_max                             | `float`  | 20                                    | Maximum distance to the preceding vehicle                                    | `"gap_max": 10`                                           |
| gap_desired                         | `float`  | 10                                    | Desired distance to the preceding vehicle                                    | `"gap_desired": 5`                                        |

### Intersection
```json
"type": "intersection_reward"
```
This reward function was made to be used in unsignalized intersection scenarios. It is composed of the following reward functions:
- Static Collisions
- Vehicle Collisions
- Speed Control
- Trajectory

Here, you can **overwrite** the default properties of these reward functions.

#### Properties
| Property                                 | Type     | Default Value | Short Description                                                                     | Usage                                             |
|------------------------------------------|----------|---------------|---------------------------------------------------------------------------------------|---------------------------------------------------|
| static_collision_factor                  | `float`  | -800          | How much vehicle collisions should be punished                                        | `"static_collision_factor": -800`                 |
| vehicle_collision_factor                 | `float`  | -600          | How much static collisions should be punished                                         | `"vehicle_collision_factor": -600`                |
| trajectory_reward_factor                 | `float`  | 1             | How much the trajectory reward should be weighted                                     | `"trajectory_reward_factor": 1`                   |
| angular_jerk_reward_factor               | `object` | 0.5           | How much angular jerk should be punishe                                               | `"angular_jerk_reward_factor": 0.5`               |
| angular_acc_reward_factor                | `float`  | 10            | How much angular acceleration should be punished                                      | `"angular_acc_reward_factor": 10`                 |
| linear_jerk_reward_factor                | `float`  | 0.05          | How much linear jerk should be punished                                               | `"linear_jerk_reward_factor": 0.05`               |
| linear_acc_reward_factor                 | `float`  | 1             | How much linear acceleration should be punished                                       | `"linear_acc_reward_factor": 1`                   |
| trajectory_following_reward              | `float`  | 1             | How much trajectory following should be rewarded                                      | `"trajectory_following_reward": 1`                |
| total_path_progress_reward_scaling       | `float`  | 10            | How much path progression should be rewarded                                          | `"total_path_progress_reward_scaling": 10`        |
| path_progress_derivative_reward_scaling  | `float`  | 100           | How much path progress derivative should be rewarded                                  | `"path_progress_derivative_reward_scaling": 100`  |
| distance_max                             | `float`  | 5             | Maximum allowed distance from the vehicle to its trajectory                           | `"distance_max": 5`                               |
| velocity_difference_scaling              | `float`  | 1             | Scaling of the reward given to the difference in vehicle velocity to desired velocity | `"velocity_difference_scaling": 1`                |
| velocity_sub_maximum_reward              | `float`  | 0.05          | Reward given to a vehicle driving slower than the maximum speed                       | `"velocity_sub_maximum_reward": 0.05`             |
| standing_punishment                      | `float`  | -10           | Punishment given to a vehicle standing still                                          | `"standing_punishment": -10`                      |
| velocity_max                             | `float`  | 40            | Maximum allowed velocity                                                              | `"velocity_max": 40`                              |
| velocity_desired                         | `float`  | 20            | Desired velocity                                                                      | `"velocity_desired": 20`                          |
| standing_threshold                       | `float`  | 0.5           | Maximum velocity of a vehicle considered to be standing still                         | `"standing_threshold": 0.5`                       |
| standing_punishment_from_step            | `int`    | 5             | Allows punishment of standing still only after some simulation steps                  | `"standing_punishment_from_step": 5`              |
| standing_punishment_to_min_path_distance | `float`  | 2.5           | Disallows punishment of standing still when close to the vehicle's goal               | `"standing_punishment_to_min_path_distance": 2.5` |

# Sum
```json
"type": "sum_reward"
```
This reward functions returns a weighted sum of other reward functions.

#### Properties

| Property        | Type       | Default Value           | Short Description                    | Usage                                       |
|-----------------|------------|-------------------------|--------------------------------------|---------------------------------------------|
| rewardFunctions | `object[]` | `{}`                    | The reward functions                 | `"rewardFunctions": [/*reward functions*/]` |
| rewardWeights   | `float[]`  | `1.0` for each function | The weights for each reward function | `"rewardWeights": [/*floats*/]`             |

