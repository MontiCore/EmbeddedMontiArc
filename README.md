# Simulation
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/coverage.svg)

This repository includes the classes for the management of the simulation, the physics computations, the environment, the sensors and the vehicles.

Discrete time steps are used to advance the the simulation.

The environment is created by parsing OpenStreetMap data.

# Vehicle Dynamics Model
A vehicle dynamics model is used to calculate the physical behaviour for car in the simulation. This VDM(Vehicle Dynamics Model) is modeled in Modelica and then compiled/exported into FMUs(Functional Mock-up Units). These units are then accesed and used in the VehicleDynamicsModel class.

To compule the FMUs, the updated modelica models have to be exported via FMI into a FMU using the OpenModelica Connection Editor v.1.12.0 (64-bit). The settings are:
Version 2.0
Type: Co-Simulation
Platform: Static

# Simulation Debugging and Debug Plotter
A use case for starting a simulation withoud starting the server is given in the firstTest jUnit test in the SimulatieVehicleTest test class.
The debug plotter can be used to visualize a simulation that contains only one physical vehicle. A use case for the debug plotter is also given in the firstTest. The plotter is registered to the simulation as a observer and is then generating one visualisation chart for each loop iteration.
