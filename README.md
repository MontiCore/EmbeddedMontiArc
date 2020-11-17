<!-- (c) https://github.com/MontiCore/monticore -->
# SimulationLanguage

SimLang is a DSL for describing settings of vehicle simulations.
In order to do so it utilizes UnitNumbers from NumberUnit and the Weather language.

SimLang files must end with .sim.

To use car models created with the [Car Modeling Language](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/CarLang), they have to be imported in the preamble of the sim file and need to be placed in their corresponding location in the model path. Then, when the scenario file is parsed, the corresponding car files will automatically loaded and are accessible from the `SimLangContainer` via the `getCarContainer()` method of the class `ExplicitVehicle`. 

## Syntax help
```
"" terms are keywords.
| denotes choices.
[]-parenthesised terms are variable input.
()? are optional inputs.
Uppercase terms relate to another rule.
```

**See the scenarios folder or test resources for examples.**

# Simulation Rules
```
import ([package].)?[carModelName];

"sim" [name] "{"
  RULE1 ";"
  RULE2 ";"
  ...
"}"

### "sim_render_frequency" [Number > 0]("ms"|"s"|"m"|"h")
Sets the simulations render frequency.
Definable once.

### "sim_loop_frequency" [Number > 0]("ms"|"s"|"m"|"h")
Sets the simulations loop frequency.
Definable once.

### "sim_duration" [Number > 0]("ms"|"s"|"m"|"h")
Sets the duration of the simulation. 
Definable once.

### "sim_type" ("fixed"|"realtime"|"maxfps")
Sets the simulations mode:
fixed: the simulation will run at a fixed number of frames
realtime: the simulation will run in relation to real time
maxfps: the simulation will run with as many frames as possible
Definable once.

### weather (FIXED|SEQUENCE|RANDOM)
#### FIXED:: "fixed" [WEATHEROBJ]
#### SEQUENCE:: "sequence" ([WEATHEROBJ] [duration:Number > 0]("ms"|"s"|"m"|"h"))+
#### RANDOM:: "random" [duration:Number > 0][ms|s|m|h]
Sets the weather behavior and settings. See the weather section below.
Definable once.

### "time" [0-23]:[0-59](:[0-59](:[0-999])?)?
Sets the daytime, at which the simulation will start.
Definable once.

### "map_path" [path]
Sets the relative path to where the map files are stored.
Definable once.

### "map_name" [name] "." [format]
Sets the map to be used. Accepted fileformats: .osm
Definable once.

### "map_height" ("flat"|"random"|(heightMap))
Sets the mode of the maps height generation.
Definable once.

### "map_overlap" [Number > 0]
Sets the size of the overlapping area of sectors.
Definable once.

### "map_sector_width" [Number > 0]
Sets the sectors width.
Definable once.
Default: 

### "map_sector_height" [Number > 0]
Sets the sectors height.
Definable once.

### "max_sector_users" [Int > 0]
Sets the maximum number of users able to be connected to a sector.
Definable once.

### "timeout" [Number > 0]("ms"|"s"|"m"|"h")
Sets the amount of idle-time until a users is auto-kicked.
Definable once.

### "gravity" [Number > 0]("m/s^2","g")
Sets the strength of the gravitational force.
Definable once.

### "pedestrian_density" [Number > 0]
Sets the amount of random pedestrians being spawned.
Definable once.

### "pedestrian" "(" [startLat] "," [startLong] ")" "->" "(" [destLat] "," [destLong] ("," [destZ])? ")"
Creates a concrete pedestrian, who follows the given path.
Definable arbitrarily often.

### "vehicle" [vehicleName] "(" [startLat] "," [startLong] "," [startRot] ")" "->" "(" [destLat] "," [destLong] ("," [destZ])? ")" (":" [fullyQualifiedCarModelName])?
Creates an instance of the given vehicle at the given coordinates, which will then follow the given path.
Definable arbitrarily often.

### "vehicle" "(" [startLat] "," [startLong] "," [startRadius] ")" "->" "(" [destLat] "," [destLong] "," [destRadius] ")" ([NumberOfVehicles])?
Creates random vehicles spawning somewhere within a circle at the given coordinates and radius, moving to somewhere within thge given destination circle.
Definable arbitrarily often.

### "vehicle" [Number > 0] ("(" [startLat] "," [startLong] "," [destLat] "," [destLong] ")")?
Creates completely random vehicle spawns and destinations with optional avoidance of a given path.
Definable arbitrarily often.

### (carModelName)? "vehicle" "{"
  "path" "(" [startLat] "," [startLong] ")" ( "->"  "(" [lat] "," [long]")" )* [destRadius]"m" ";"
  "randomPath"; // if randomPath is set, then "path" will be ignored
  "network" [networkType];
  "goals" "[" (unaryGoal|binaryGoal)+ "]" ";"
  "platoon" "{" 
    "size" [Number > 0] ";"
    // or describing multiple scenarios with different platoon sizes
    "size" [startSize:endSize:step] ";"
  "}"
"}"
#### unaryGoal = ["never" | "always" | "eventually" |] [metricName] ["gt" | "ge" | "lt" | "le" | "eq" ] [targetValue][unit];
#### binaryGoal = unaryGoal "until" unaryGoal;
#### metricName = ["speed" | "acceleration" | "batteryLevel"];
Creates a vehicle or a platoon. A vehicle will spawn at the first way point defined in its path attribute(sartLat, startLong).
If "platoon" is given, the rest vehicles will spawn following the leading vehicle.
A platoon is considered reached its destination if the leading vehicle reached the destination
while the rest of the vehicles are still in the platoon.
A vehicle can have multiple "goals", they should be fullfilled for a successful simulation.
Each condition should be given a type: "never", "always", "eventually" or "until".
The binary goal "until" is special, its basic form is: cond1 "until" cond2, meaning cond1 should always be true until cond2 is true.
It can be used to test scenarios like: 
an EV should not release too much power until battery level is above some level, otherwise it might damage the battery("batteryLevel" ">" 0.3 "until" "acceleration" ">" 25 km/s^2);
or:
a vehicle should stay still until the traffic light turns green("speed" "==" 0 km/s "until" "green").

### ("fixed"|"bound") "channel" [name] "{"
  CHANNEL_RULE1 ";"
  CHANNEL_RULE2 ";"
"}"
Create a communication-channel existing in the simulation. See Communication Configuration for rules.
Definable arbitrarily often, but channel names must be unique.
```
# Communication/Channel Rules
```
### "transfer_rate" [Number > 0]
Defines the transfer rate of this channel.

### "latency" [Number > 0]
Defines the latency of this channel.

### "outage" [Number [0,1]]
Defines the probabilty of an outage of this channel.

### "area" ("global"| [Coordinate] ([Coordinate]|[radius]))
Defines the channels area of effect. Either global or definable in a rectangle or circle.
```
# Weather Rules
```
### WEATHEROBJ:: "{" RULE1 RULE2 ... "}"

### "temperature" [Number]("K"|"°C°|"°F")
Set the temperature.
Definable once.
Default: 20°C

### "humidity" [Number]
Sets the humidity. Either as float in [0-1].
Definable once.

### "pressure" [Number > 0]"Pa"
Sets the pressure.
Definable once.

### "wind_strength" [Number > 0]("m/s"|"km/h"|"mph")
Sets the wind strength.
Definable once.

### "wind_direction" [0 <= Number < 360]"°"
Sets the wind direction
Definable once.

### "precipitation_type" ("none"|
                          "drizzle"|
                          "rain"|
                          "freezingdrizzle"|
                          "freezingrain"|
                          "snowrain"|"snain"|
                          "snow"|
                          "snowgrains"|
                          "icepellets"|"sleet"|
                          "hail"|
                          "snowpellets"|"graupel"|
                          "icecrystals")
Sets the precipitation type.
Definable once.

### "precipitation_amount" [Number >= 0] ("l/m^2"|"mm")
Sets the precipitation amount.
Definable once.

### "clouding" ("none"|
               "cirrostratus"|
               "altostratus"|
               "stratus"|
               "nimbostratus"|
               "noctilucent"|
               "polarstratospheric"|
               "cirrus"|
               "cirrocumulus"|
               "altocumulus"|
               "stratocumulus"|
               "cumulushumilis"|
               "cumulusmediocris"|
               "cumuluscongestus"|
               "cumulonimbus")
Sets the type of clouding.
Definable once.

### "sight" ["unlimited"|([Number > 0]["mm"|"cm"|"m"|"km")]
Sets the range of view/sight distance.
Definable once.

### "weather_phenomena" ("fog"|
                         "ropetornado"|
                         "conetornado"|
                         "wedgetornado"|
                         "multivortextornado"|
                         "landspout"|
                         "waterspout"|
                         "gustnado"|
                         "dustdevil"|
                         "steamdevil"|
                         "thunderstorm") ("(" (posX) "," (posY) ")" )?
Creates an instance of the given phenomena at the optional coordinates.
Definable arbitrarily often.

### "optical_phenomena" ("rainbow"|
                         "northernlights"|
                         "circumzenithalarc"|
                         "zodiacallight"|
                         "crepuscularrays"|
                         "mirage"|
                         "fogbow")
Creates an instance of the given phenomena.
Definable arbitrarily often.

### "artificial_phenomena" ("contrails"|"smog"|"rocketexhausttrails")
Creates an instance of the given phenomena.
Definable arbitrarily often.
```
