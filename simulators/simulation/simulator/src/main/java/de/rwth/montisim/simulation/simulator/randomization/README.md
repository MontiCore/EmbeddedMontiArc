# Randomization

The randomization is used to create fully randomized scenarios.
There are different strategies for the randomization, all of which creating scenarios for a specific purpose.

## Usage
In the [basic simulator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator) the randomization can be specified inside the scenario files as a list of randomization strategy properties:
```json
{
  ...,
  "randomization" : [
    {
      "type": "basic"
    },
    {
      "type": "platooning",
      "minNumberOfVehicles": 5,
      "maxNumberOfVehicles": 10 // <- Stragegy properties are optional. If not present, default will be used
    },
    ...
  ],
  ...
}
```

## Randomization Strategies:
### Basic Strategy
```json
"type": "basic"
```
The basic strategy creates a scenario in which a single vehicle has to follow a path of nodes.

It was designed to be used in training the basic driving skills of an autopilot.

#### Properties
| Property        | Type     | Default Value | Short Description          | Usage                     |
|-----------------|----------|---------------|----------------------------|---------------------------|
| minGoalDistance | `double` | 500           | Minimum length of the path | `"minGoalDistance": 500`  | 
| maxGoalDistance | `double` | 1000          | Maximum length of the path | `"maxGoalDistance": 1000` | 


### Platooning Strategy
```json
"type": "platooning"
```
The platooning strategy creates a scenario in a number of vehicles should follow a path of nodes.
The vehicles are not necessarily all spawned directly behind each other, but are rather spawned on the streets outgoing of the first point of the path in a tree fashion.
In a similar fashion, the end positions of the vehicles are generated.

This is done to encourage the vehicles forming their own platoons and in the end splitting from platoons.

#### Properties
| Property               | Type     | Default Value | Short Description                                        | Usage                          |
|------------------------|----------|---------------|----------------------------------------------------------|--------------------------------|
| minNumberOfVehicles    | `int`    | 2             | Minimum number of vehicles                               | `"minNumberOfVehicles": 2`     | 
| maxNumberOfVehicles    | `int`    | 20            | Maximum number of vehicles                               | `"maxNumberOfVehicles": 20`    | 
| minDistanceBtwVehicles | `double` | 10            | Minimum distance between the spawn positions of vehicles | `"minDistanceBtwVehicles": 10` | 
| maxDistanceBtwVehicles | `double` | 20            | Maximum distance between the spawn positions of vehicles | `"maxDistanceBtwVehicles": 20` | 
| minGoalDistance        | `double` | 500           | Minimum length of the path                               | `"minGoalDistance": 500`       | 
| maxGoalDistance        | `double` | 1000          | Maximum length of the path                               | `"maxGoalDistance": 20`        | 


### Intersection Strategy

The intersection strategy creates scenarios in which vehicles have to cross a common intersection to reach their goal.
This encourages the agent to learn a generalizable policy.

#### Properties
| Property               | Type     | Default Value | Short Description                                        | Usage                          |
|------------------------|----------|---------------|----------------------------------------------------------|--------------------------------|
| minNumberOfVehicles    | `int`    | 1             | Minimum number of vehicles                               | `"minNumberOfVehicles": 2`     | 
| maxNumberOfVehicles    | `int`    | 1             | Maximum number of vehicles                               | `"maxNumberOfVehicles": 20`    | 
| minDistanceBtwVehicles | `double` | 8             | Minimum distance between the spawn positions of vehicles | `"minDistanceBtwVehicles": 10` | 
| maxDistanceBtwVehicles | `double` | 8             | Maximum distance between the spawn positions of vehicles | `"maxDistanceBtwVehicles": 20` | 
| minGoalDistance        | `double` | 5             | Minimum length of the path                               | `"minGoalDistance": 500`       | 
| maxGoalDistance        | `double` | 5             | Maximum length of the path                               | `"maxGoalDistance": 20`        | 

