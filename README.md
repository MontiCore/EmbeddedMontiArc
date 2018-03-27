[![Build Status](https://travis-ci.org/MontiSim/SimLang.svg?branch=master)](https://travis-ci.org/MontiSim/SimulationLanguage)
<a href="https://codeclimate.com/github/MontiSim/SimLang/maintainability"><img src="https://api.codeclimate.com/v1/badges/dc74fd9002ab989462b0/maintainability" /></a>
<a href="https://codeclimate.com/github/MontiSim/SimLang/test_coverage"><img src="https://api.codeclimate.com/v1/badges/dc74fd9002ab989462b0/test_coverage" /></a>
[![Coverage Status](https://coveralls.io/repos/github/MontiSim/SimLang/badge.svg?branch=master)](https://coveralls.io/github/MontiSim/SimLang?branch=master)

# SimulationLanguage

SimLang is a DSL for describing settings of vehicle simulations.
In order to do so it utilizes TNumberUnits from NumberUnit and the Weather language.

SimLang files must end with .sim.

## Syntax help
```
"" terms are keywords.
| denotes choices.
[]-parenthesised terms are variable input.
()? are optional inputs.
Uppercase terms relate to another rule.
```

**See FullExample.sim for a concrete example of each rule.**

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
Default: 60ms

### "sim_loop_frequency" [Number > 0]("ms"|"s"|"m"|"h")
Sets the simulations loop frequency.
Definable once.
Default: 60ms

### "sim_duration" [Number > 0]("ms"|"s"|"m"|"h")
Sets the duration of the simulation. 
Definable once.
Default: 1h

### "sim_type" ("fixed"|"realtime"|"maxfps")
Sets the simulations mode:
fixed: the simulation will run at a fixed number of frames
realtime: 
maxfps: the simulation will run with as many frames as possible
Definable once.
Default: fixed 30

### weather (FIXED|SEQUENCE|RANDOM|~~FORECAST~~)
#### FIXED:: "fixed" [WEATHEROBJ]
#### SEQUENCE:: "sequence" [WEATHEROBJ]+ [duration:Number > 0]("ms"|"s"|"m"|"h"))?
#### RANDOM:: "random" [duration:Number > 0][ms|s|m|h]
#### ~~FORECAST:: "forecast" [WEATHEROBJ] [duration] (automaton)~~ *Not implemented yet*
Sets the weather behavior and settings. See the weather section below.
Definable once.
Default: fixed {}

### "time" [0-23]:[0-59](:[0-59](:[0-999])?)?
Sets the daytime, at which the simulation will start.
Definable once.
Default: 0:0

### "map_path" [path]
Sets the relative path to where the map files are stored.
Definable once.
Default: Maps

### "map_name" [name]
Sets the map to be used. Accepted fileformats: .osm
Definable once.
Default:

### "map_height" ("flat"|"random"|(heightMap))
Sets the mode of the maps height generation.
Definable once.
Default: flat

### "map_overlap" [Number > 0]
Sets the size of the overlapping area of sectors.
Definable once.
Default:

### "map_sector_width" [Number > 0]
Sets the sectors width.
Definable once.
Default: 

### "map_sector_height" [Number > 0]
Sets the sectors height.
Definable once.
Default:

### "max_sector_users" [Int > 0]
Sets the maximum number of users able to be connected to a sector.
Definable once.
Default:

### "timeout" [Number > 0]("ms"|"s"|"m"|"h")
Sets the amount of idle-time until a users is auto-kicked.
Definable once.
Default:

### "gravity" [Number > 0]("m/s^2")
Sets the strength of the gravitational force.
Definable once.
Default: 9.81m/s^2

### "pedestrian_density" [Number > 0]
Sets the amount of random pedestrians being spawned.
Definable once.
Default: 1.0

### "<p>" "(" [startX] "," [startY] ")" "->" "(" [destX] "," [destY] ("," [destZ])? ")"
Creates a concrete pedestrian, who follows the given path.
Definable arbitrarily often.
Default: 

### [vehicleName] "(" [startX] "," [startY] "," [startRot] ")" "->" "(" [destX] "," [destY] ("," [destZ])? ")"
Creates an instance of the given vehicle at the given coordinates, which will then follow the given path.
Definable arbitrarily often.
Default: 

### "<v>" "(" [startX] "," [startY] "," [startRadius] ")" "->" "(" [destX] "," [destY] "," [destRadius] ")" ([NumberOfVehicles])?
Creates random vehicles spawning somewhere within a circle at the given coordinates and radius, moving to somewhere within thge given destination circle.
Definable arbitrarily often.
Default: 

### "<v>" [Number > 0] ("(" [startX] "," [startY] "," [destX] "," [destY] ")")?
Creates completely random vehicle spawns and destinations with optional avoidance of a given path.
Definable arbitrarily often.
Default:

### ("fixed"|"bound") "channel" [name] "{"
  CHANNEL_RULE1 ";"
  CHANNEL_RULE2 ";"
"}"
Create a communication-channel existing in the simulation. See Communication Configuration for rules.
Definable arbitrarily often, but channel names must be unique.
Default:
```
# Communication/Channel Rules
```
### "transferrate" [Number > 0]
Defines the transferrate of this channel.

### "latency" [Number > 0]
Defines the latency of this channel.

### "outage" [Number 0:1 | Number 0:100 "%"]
Defines the probabilty of an outage of this channel.

### "area" ("global"| [Coordinate] ([Coordinate]|[radius]))
Defines the channels area of effect. Either global or defineable in a rectangle or circle.
```
# Weather Rules
```
### WEATHEROBJ:: "{" RULE1 RULE2 ... "}"

### "temperature" [Number]("K"|"°C°|"°F")
Set the temperature.
Definable once.
Default: 20°C

### "humidity" [Number] ("%")?
Sets the humidity. Either as float in [0-1] or in % in [0-100].
Definable once.
Default: 0.1

### "pressure" [Number > 0]"Pa"
Sets the pressure.
Definable once.
Default: 1Pa

### "windstrength" [Number > 0]("m/s"|"km/h"|"mph")
Sets the windstrength.
Definable once.
Default: 0km/h

### "winddirection" [0 <= Number < 360]"°"
Sets the winddirection
Definable once.
Default: 0

### "precipitation_type" ("none"|
                          "drizzle"|
                          "rain"|
                          "freezing drizzle"|
                          "freezing rain"|
                          "snow rain"|"snain"|
                          "snow"|
                          "snow_grains"|
                          "ice_pellets"|"sleet"|
                          "hail"|
                          "snow_pellets"|"graupel"|
                          "ice_crystals")
Sets the precipitation type.
Definable once.
Default: none

### "precipitation_amount" [Number >= 0] ("l/m^2"|"mm")
Sets the precipitation amount.
Definable once.
Default: 

### "clouding" ("none"|
               "cirrostratus"|
               "altostratus"|
               "stratus"|
               "nimbostratus"|
               "noctilucent"|
               "polar stratospheric"|
               "cirrus"|
               "cirrocumulus"|
               "altocumulus"|
               "stratocumulus"|
               "cumulus_humilis"|
               "cumulus_mediocris"|
               "cumulus_congestus"|
               "cumulonimbus")
Sets the type of clouding.
Definable once.
Default: none

### "sight" ["unlimited"|([Number > 0]["mm"|"cm"|"m"|"km")]
Sets the range of view/sight distance.
Definable once.
Default: unlimited

### "weather_phenomena" ("fog"|
                         "rope_tornado"|
                         "cone_tornado"|
                         "wedge_tornado"|
                         "multi-vortex_tornado"|
                         "landspout"|
                         "waterspout"|
                         "gustnado"|
                         "dust_devil"|
                         "steam_devil"|
                         "thunderstorm") ("(" (posX) "," (posY) ")" )?
Creates an instance of the given phenomena at the optional coordinates.
Definable arbitrarily often.
Default: 

### "optical_phenomena" ("rainbow"|
                         "northern_lights"|
                         "circumzenithal_arc"|
                         "zodiacal_light"|
                         "crepuscular_rays"|
                         "mirage"|
                         "fog_bow")
Creates an instance of the given phenomena.
Definable arbitrarily often.
Default: 

### "artificial_phenomena" ("contrails"|"smog"|"rocket_exhaust_trails")
Creates an instance of the given phenomena.
Definable arbitrarily often.
Default: 
```
