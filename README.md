# Simulation
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/coverage.svg)

This repository includes the classes for the management of the simulation, the physics computations, the environment, the sensors and the vehicles.

Discrete time steps are used to advance the simulation.

The environment is created by parsing OpenStreetMap data.

# Vehicle Dynamics Model

A vehicle dynamics model is used to calculate the physical behaviour for a car in the simulation. This vehicle dynamics model is modeled in Modelica and then compiled into functional mock-up units. These units are then accessed and used in the `VehicleDynamicsModel` class.

To compile updated Modelica models into functional mock-up units, the _.mo_ files (the models) have to be exported via FMI into a _.fmu_ file using the OpenModelica Connection Editor v.1.12.0 (64-bit).

__NOTE:__ When new fmu's are added to the simulation, the _deployment.bat_ that copies the fmu's into the SmartFoxServer has to be updated as well.

The settings are:
* Version: 2.0
* Type: Co-Simulation
* Platform: Static

# Simulation Debugging and Debug Plotter

To quickly debug the simulator, a simulation can be configured and executed without starting up the server. An example is given in the `firstTest()` jUnit test in the `SimulatieVehicleTest` test class.

The debug plotter can be used to visualize a simulation that contains only __one__ physical vehicle. A use case for the debug plotter is also given in `firstTest()`. The plotter is registered to the simulation as an observer and is then generating one visualisation chart for each loop iteration.

# Height data
Height data is read from SRTM files, which are documented [here](https://dds.cr.usgs.gov/srtm/version2_1/Documentation/SRTM_Topo.pdf). Additional height data files can be downloaded from [this source](https://dds.cr.usgs.gov/srtm/version2_1/SRTM3/), where you need to select the continent and then search for the file you need. The file name format is dependent on latitude/longitude and defined as `[N/S][<LAT>][O/W][<LONG>].hgt`.

Each file contains data until the next higher latitude/longitude. For example file N50E006.hgt will cover all pairs of (latitude,longitude) from (50,6) to (51,7) (with resolution 3/3600=0.000833).

To determine min/max latitude/longitude, the file names of all loaded height data files need to be parsed according to the described format and then compared to find the lowest/highest values. Let's say N50E006.hgt and N51E006.hgt are loaded, then you would get minLat=50, minLong=6, maxLat=52, maxLong=7.
