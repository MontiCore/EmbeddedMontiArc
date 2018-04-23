[![Build Status](https://travis-ci.org/MontiSim/SimulationLanguage.svg?branch=master)](https://travis-ci.org/MontiSim/SimulationLanguage)
<a href="https://codeclimate.com/github/MontiSim/SimulationLanguage/maintainability"><img src="https://api.codeclimate.com/v1/badges/dc74fd9002ab989462b0/maintainability" /></a>
<a href="https://codeclimate.com/github/MontiSim/SimulationLanguage/test_coverage"><img src="https://api.codeclimate.com/v1/badges/dc74fd9002ab989462b0/test_coverage" /></a>
[![Coverage Status](https://coveralls.io/repos/github/MontiSim/SimulationLanguage/badge.svg?branch=master)](https://coveralls.io/github/MontiSim/SimulationLanguage?branch=master)

# SimulationLanguage

SimLang is a DSL for describing settings of vehicle simulations.
In order to do so it utilizes UnitNumbers from NumberUnit and the Weather language.

SimLang files must end with .sim.

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

### "<p>" "(" [startX] "," [startY] ")" "->" "(" [destX] "," [destY] ("," [destZ])? ")"
Creates a concrete pedestrian, who follows the given path.
Definable arbitrarily often.

### [vehicleName] "(" [startX] "," [startY] "," [startRot] ")" "->" "(" [destX] "," [destY] ("," [destZ])? ")"
Creates an instance of the given vehicle at the given coordinates, which will then follow the given path.
Definable arbitrarily often.

### "<v>" "(" [startX] "," [startY] "," [startRadius] ")" "->" "(" [destX] "," [destY] "," [destRadius] ")" ([NumberOfVehicles])?
Creates random vehicles spawning somewhere within a circle at the given coordinates and radius, moving to somewhere within thge given destination circle.
Definable arbitrarily often.

### "<v>" [Number > 0] ("(" [startX] "," [startY] "," [destX] "," [destY] ")")?
Creates completely random vehicle spawns and destinations with optional avoidance of a given path.
Definable arbitrarily often.

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
