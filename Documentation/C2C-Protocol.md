# Car-to-Car Communication Protocol

[[_TOC_]]

## General Information

Each vehicle sends position, heading and velocity information in every time step as a broadcast to every other vehicle.
-  `Q^2 position`
-  `Q heading`
-  `Q velocity`

Thereby previously planned scenarios like object detection and emergency braking do not need specialized packets.

## Packet Structure

### Subject

Every message consists of at least a subject which indicates the purpose of the message.
Valid values of this parameter are:
1.  `KEEP_SAFETY_DISTANCE`
2.  `EMERGENCY_CORRIDOR`
3.  `CREATE_PLATOON`
4.  `JOIN_PLATOON`
5.  `CONFIRM_PLATOON`
6.  `DECLINE_PLATOON`
7.  `LEAVE_PLATOON`
8.  `OVERTAKE`
9.  `DELAY_REQUEST`
10.  `DELAY_WAITING`
11.  `DELAY_GATHERING_POINT`
12.  `DESTINATION_REACHED`
13.  `PARK`
14.  `REQUEST_PARKING_PLACE`
15.  `FOUND_PARKING_PLACE`
16.  `BATTERY_CHARGE`
17.  `REQUEST_FILL_BATTERY`
18.  `FILLING_BATTERY`
19.  `SCAN_INTERSECTION`
20.  `TRAFFIC_JAM`
21.  `WEATHER_CONDITIONS`


### Optional flags
-  Authority (e.g. emergency, police)

## Packet types
The different Subjects are transmitted in as packet.  If no other information than the subject itself is required, the `SubjectPacket` is used. If more information is necessary the `SubjectPacket` is replaced with a specialized packet.

### SubjectPacket

-  `Subject` (see [list](#subject) above, only valid for subject no. 2, 3, 5, 6, 7, 9, 10, 12, 14, 16, 20)

### SafetyDistancePacket

-  `Subject = KEEP_SAFETY_DISTANCE`
-  `ActionType action` (enum: brake, accelerate)
-  `Q targetVelocity`

### PlatoonPacket

-  `Subject` (see [list](#subject) above, only valid for subject no. 4, 5)
-  `N1 index` (of the vehicle in the platoon)

### BatteryLevelPacket

-  `Subject = BATTERY_CHARGE`
-  `Q batteryLevel`

### LocationPacket

-  `Subject` (see [list](#subject) above, only valid for subject no. 11, 13, 15)
-  `Q^2 location`

### UnsignalizedIntersectionPacket

-  `Subject = SCAN_INTERSECTION`
-  `Q^2 intersection position`
-  `Q intended direction`
-  `Q current distance to intersection`

### WeatherConditionPacket

-  `Subject = WEATHER_CONDITIONS`
-  `WeatherCondition (enum: normal, rain, snow, ice)`

### SwitchingLanePacket

-  `Subject = OVERTAKE`
-  `N1 laneIndex`

### PricePacket

-  `Subject = FILLING_BATTERY`
-  `Q price`

## Scenarios

_Note: Requirements such as ***Object Detection***, ***Emergency Braking*** or ***Platoon Synchronization*** are handled by using general information._ 


### Safety Distance (REQ-3.4)

To keep the safety distance in a more comfortable way, a car can inform the cars in behind to break/accelerate by sending a `SafetyDistancePacket` (`Subject = KEEP_SAFETY_DISTANCE`).

### Platooning (REQ-5)

The following scenarios for **Platooning** also apply here.

#### Create platoon

Assuming we know the identification of the car we want to create a platoon with. 
To create a platoon a connection is established using this Three-Way-Handshake initiated with a `SubjectPacket` with subject `CREATE_PLATOON`.

The answer would be a SubjectPacket with subject `CONFIRM_PLATOON/DECLINE_PLATOON` and both parties have to confirm the platoon creation.

#### Joining a platoon

In order to join a platoon a vehicle needs to send a `PlatoonPacket` with subject `JOIN_PLATOON`. 
There will be response containing either a `PlatoonPacket` with subject CONFIRM_PLATOON and the index to join on or a `SubjectPacket` with subject `DECLINE_PLATOON`.

#### Leaving a platoon

To leave a platoon, a vehicle just notifies the other members with a `SubjectPacket` and the subject `LEAVE_PLATOON`.

#### Delay

Car(s) inform(s) the lead car about a sudden delay (e.g. accident occurred, road is blocked). 
Leading car decides if the platoon waits for the issue to be resolved or sends the delayed car(s) the coordinates of a gathering point on the trajectory that is reachable over an alternative route where the platoon waits.
At first a vehicle sends a `SubjectPacket` with subject `DELAY_REQUEST` which will be confirmed using either a `SubjectPacket` with subject `DELAY_WAITING` or with a `LocationPacket` with subject `DELAY_GATHERING_POINT` including the gathering location.

Note: Waiting cars are waiting on an appropriate spot like a parking spot or the roadside (Seitenstreifen)

#### Park Platoon (REQ-15)

The leading car instructs the other cars to find an appropriate parking spot nearby by sending a `LocationPacket` with subject `PARK`.

#### Destination Reached

The leading car informs the other cars that the destination is reached using a `SubjectPacket` with subject `DESTINATION_REACHED`.

#### Battery Charge (REQ-12, REQ-14)

Car with a battery charge that is too low to reach the destination informs other cars that it needs to charge in order to be able to reach the destination.
It thereby sends a `BatteryLevelPacket` with subject `BATTERY_CHARGE`.

### Unsignalized intersection (REQ-6.1)

When an unsignalized intersection is detected, a vehicle sends a `UnsignalizedIntersectionPacket` (`subject = SCAN_INTERSECTION`) to scan for other cars.

If our car has to stop, resend the message above in a specific interval.

### Traffic jam detection (REQ-7)

Every car that stands still on a street sends a warning `SubjectPacket` with subject `TRAFFIC_JAM` to other cars.

### Weather Condition Sensor (REQ-8.1)

If a car detects special weather conditions, it should send a `WeatherConditionPacket` (`subject = WEATHER_CONDITIONS`) inform/warn all the others behind it.

### Emergency corridor (REQ-9)

Emergency vehicle transmits the subject `EMERGENCY_CORRIDOR` inside a `SubjectPacket`.

### Switching Lanes (REQ-11)

In the situation that a car wants to overtake another car, it should send a `SwitchingLanePacket` to notify all the cars around not to use the same lane (`subject = OVERTAKE`).

### PricePacket

-  `Subject = FILLING_BATTERY`
-  `Q price`

_Note: The `PricePacket` cannot be implemented in the current version of EMA_


### Filling up the Battery at a Station (REQ-14)

When a vehicle arries at a station without passengers in it, it sends a request to fill the battery to the station (`PaymentInformationPacket` with `subject = REQUEST_FILL_BATTERY`).

A response would be a `PricePacket` (`subject = FILLING_BATTERY`).


### Parking (REQ-15)

To detect empty parking slots, the vehicle requests the closest position of a slot by sending a `SubjectPacket` (`subject = REQUEST_PARKING_PLACE`).

A response would be a `LocationPacket` (`subject = FOUND_PARKING_PLACE`).
