# Preprocessing

When training agents it is often useful to have more control over the communication between theirs states.
In some cases for example we don't need every published state or need to clean up some variables.

Preprocessors work on the published state of a vehicle and the state packets of the other vehicles.
They allow us to normalize, centralize or filter out certain states.
## Usage
In the [basic simulator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator) we can specify
the preprocessor with the preprocessor property in our scenario file:
```json
{
  ...,
  "preprocessor" : {
    "type": "proximity",
    "maxNumberOfVehicles": 3,
    "addIndicator": true
  },
  ...
}
```

## Preprocessing Strategies:
### Default Preprocessor
```json
"type": "default"
```
The default preprocessor concatenates the state of the vehicle together with the state packets of the
other vehicles.
This results in a state that is dependent on the number of vehicles in the simulation.
If the preprocessor isn't specified, this strategy will be used.
The default preprocessor does not have any properties.


### Proximity Filter
```json
"type": "proximity"
```
The Proximity Filter only filters out all other vehicles' states except for the closest N vehicles. 
If there are less than N vehicles, their remaining states are filled with zeros.
Additionally, the indicator flag can be set to indicate empty or full state packets.

This preprocessor, other than the default preprocessor, returns a state array of constant size.

The size of the state without the indicator is: vehicleState + statePacketLength * maxNumberOfVehicles

The size of the state with the indicator is: vehicleState + (statePacketLength + 1) * maxNumberOfVehicles

#### Properties
| Property            | Type      | Default Value | Short Description                | Usage                      |
|---------------------|-----------|---------------|----------------------------------|----------------------------|
| maxNumberOfVehicles | `int`     | 5             | Number of State Packets          | `"maxNumberOfVehicles": 3` | 
| addIndicator        | `boolean` | true          | Add flag for empty state packets | `"addIndicator": false`    | 

### Sequence Preprocessor
```json
"type": "sequence"
```
The Sequence Preprocessor allows using multiple preprocessor in a sequence.
#### Properties
| Property      | Type             | Default Value | Short Description                | Usage                                     |
|---------------|------------------|---------------|----------------------------------|-------------------------------------------|
| preprocessors | `Preprocessor[]` | empty         | List of Preprocessors            | `"preprocessors": [{ "type": "default"}]` | 

### Angle Preprocessor
```json
"type": "angle"
```
The Angle Preprocessor replaces angles in the state with their sin-cos values.
Thus, we replace each angle with two values from -1 to 1.
#### Properties
| Property              | Type  | Default Value | Short Description                           | Usage                         |
|-----------------------|-------|---------------|---------------------------------------------|-------------------------------|
| angleStateIndex       | `int` | 23            | Index of the angle in the vehicle state     | `"angleStateIndex": 7`        | 
 | angleStatePacketIndex | `int` | 2             | Index of the angle in the statepacket state | `"angleStatePacketIndex": 13` |

### Trajectory Filter
```json
"type": "trajectory"
```
The Trajectory Filter reorders all other vehicles' states depending on, if they are likely to collide with the current vehicle in the near future.

For this, Kalman Filtering is used to approximate the future locations of vehicles, depending on their trajectory, if they would drive with the current velocity constantly.
Vehicles, that are likely to collide soon with the current vehicle, have a high priority.

Similarly to the Proximity Filter, the Trajectory Filter also returns a state array of constant size and can add an indicator flag.

#### Properties
| Property               | Type      | Default Value | Short Description                                             | Usage                          |
|------------------------|-----------|---------------|---------------------------------------------------------------|--------------------------------|
| VEHICLE_WIDTH          | `float`   | 1.87          | Width of a vehicle                                            | `"VEHICLE_WIDTH": 2`           | 
| VEHICLE_LENGTH         | `float`   | 4.871         | Height of a vehicle                                           | `"VEHICLE_LENGTH": 5`          |
| MAX_STATES_PER_VEHICLE | `int`     | 25            | Maximum amount of future states to be generated for a vehicle | `"MAX_STATES_PER_VEHICLE": 10` |
| maxNumberOfVehicles    | `int`     | 5             | Number of State Packets                                       | `"maxNumberOfVehicles": 3`     | 
| addIndicator           | `boolean` | true          | Add flag for empty state packets                              | `"addIndicator": false`        | 

