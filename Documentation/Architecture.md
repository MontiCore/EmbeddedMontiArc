# Architecture 

[[_TOC_]]

## Architecture Diagram
![Autopilot Architecture Diagram](Documentation/ComponentDiagram.png)

## I/O

### Input
* Camera
* Battery
* Lidar
* Weather
* OSM
* Speedometer
* Compass
* GPS
* Communcation Interface

### Output
* Steering Wheel
* Gas Pedal
* Brake Pedal
* Communcation Interface


## Control Unit
Main component that controls all inputs, outputs and inter-controller communication of the autopilot.


## IIR Filter
Implementation of an IIR filter for denoising of sensor values.

**Related Requirements**
* REQ-2.3 Comfortable Acceleration


## Environment Controller
Controller for managing environmental influences. 

### External Sensors
Sensors that can read environment data. 

**Related Requirements**
* REQ-6.1 Unsignalized Intersection
* REQ-6.2 Signalized Intersection
* REQ-9 Emergency Corridor
* REQ-11 Switching Lanes
* REQ-15 Parking

#### Obstacle Detector
Detector for obstacles in the environment via the Lidar input. Uses the IIR Filter component to correct values.

**Related Requirements**
* REQ-3.2 Object Detection

#### Weather Detector
Detector for weather conditions. Uses the IIR Filter component to correct values.

**Related Requirements**
* REQ-8.1 Weather Condition Sensor

### Vehicle Stats
Component for access to the vehicle stats e.g. tank level or battery charge.

**Related Requirements**
* REQ-12 Battery Level
* REQ-13 Regenerative Braking
* REQ-14 Filling up the Battery at a Station


## Navigation Controller
Controller for vehicle navigation. Uses input from speedometer and compass. Uses IIR Filter to correct values for every subcomponent.

### Path Keeper
Component that monitors if the vehicle is still on the computed path to destination.

**Related Requirements**
* REQ-1.1 Starting and Moving a Vehicle in the Simulator
* REQ-1.2 Following A Path

### Open Street Map Adapter
Pulls necessary information from Open Street Map API. 

**Related Requirements**
* REQ-6.1 Unsignalized Intersection
* REQ-6.2 Signalized Intersection
* REQ-11 Switching Lanes
* REQ-14 Filling up the Battery at a Station
* REQ-15 Parking

### Pathfinder
Processes OSM data and computes a driving path to destination aided by GPS input.

**Related Requirements**
* REQ-1.1 Starting and Moving a Vehicle in the Simulator
* REQ-1.2 Following A Path
* REQ-10 Trajectory Planning
* REQ-11 Switching Lanes
* REQ-14 Filling up the Battery at a Station
* REQ-15 Parking


## Communication Controller
Controller for Communication with other devices. Uses IIR Filter to correct values for every subcomponent.

### Vehicle Com
Component for communication with other vehicles. Uses Negotiator and Scanner component.

**Related Requirements**
* REQ-3.4 Safety Distance
* REQ-5 Platooning
* REQ-6.1 Unsignalized Intersection
* REQ-7 Traffic Jam Detection
* REQ-9 Emergency Corridor
* REQ-10 Trajectory planning

### Generic Com
Component for generic communication with non-vehicle devices. Uses Negotiator and Scanner component.

**Related Requirements**
* REQ-6.1 Unsignalized Intersection
* REQ-6.2 Signalized Intersection
* REQ-15 Parking

### Negotiator
Component for negotiating terms and constraints with other vehicles or devices.

**Related Requirements**
* REQ-5 Platooning
* REQ-6.1 Unsignalized Intersection

### Scanner
Component for scanning the nearby room for other devices or vehicles that are able to communicate.

**Related Requirements**
* REQ-3.4 Safety Distance
* REQ-5 Platooning
* REQ-9 Emergency Corridor
* REQ-11 Switching Lanes
* REQ-15 Parking


## Behaviour Controller
Controller for driving behaviour of the autopilot.

### Constraints
Component for implementation of traffic rules and vehicle boundaries.

**Related Requirements**
* REQ-2.1 Upper Speed Limit
* REQ-2.2 Minimum Speed Limit
* REQ-3.4 Safety Distance
* REQ-6.1 Unsignalized Intersection
* REQ-6.2 Signalized Intersection
* REQ-9 Emergency Corridor

### Driving Profile
Component for style of driving. E.g. active/passive maneuvers, aggressiveness, individual preferences for maximum speed/steering angles.

**Related Requirements**
* REQ-2.1 Upper Speed Limit
* REQ-2.3 Comfortable Acceleration
* REQ-8 Adapting To Weather Conditions
* REQ-11 Switching Lanes

### Safety
Component for evaluating and acting in dangerous situations that can override other behaviour preferences.

**Related Requirements**
* REQ-3.1 Braking
* REQ-3.2 Object Detection
* REQ-3.3 Braking Distance Calculation
* REQ-3.4 Safety Distance
* REQ-3.5 Emergency braking

## Movement Controller
Controller for controlling the vehicle movement hardware.

### Steering
Component for controlling the steering wheel aided by the PID/MPC component.

**Related Requirements**
* REQ-1.1 Starting and Moving a Vehicle in the Simulator
* REQ-1.2 Following A Path
* REQ-4 Turning

### PID/MPC
Component for smoothing values.

**Related Requirements**
* REQ-2.3 Comfortable Acceleration

### Velocity
Component for controlling the velocity of the vehicle by using the gas pedal and brake pedal aided by the PID/MPC component.

**Related Requirements**
* REQ-1.1 Starting and Moving a Vehicle in the Simulator
* REQ-1.2 Following A Path
* REQ-3.1 Braking