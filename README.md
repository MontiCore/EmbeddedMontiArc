# Simulation
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/coverage.svg)

This repository includes the classes for the management of the simulation, the physics computations, the environment, the sensors and the vehicles.

Discrete time steps are used to advance the the simulation.

The environment is created by parsing OpenStreetMap data.

# Vehicle Dynamics Model

A vehicle dynamics model is used to calculate the physical behaviour for car in the simulation. This vehicle dynamics model is modeled in Modelica and then compiled into functional mock-up units. These units are then accesed and used in the `VehicleDynamicsModel` class.

To compile updated modelica models into functional mock-up units, the _.mo_ files (the models) have to be exported via FMI into a _.fmu_ file using the OpenModelica Connection Editor v.1.12.0 (64-bit).

The settings are:
 * Version: 2.0
 * Type: Co-Simulation
 * Platform: Static

# Simulation Debugging and Debug Plotter

To quickly debug the simulatro, a simulation can be configured and executed withoud starting up the server. An example is given in the `firstTest` jUnit test in the `SimulatieVehicleTest` test class.

The debug plotter can be used to visualize a simulation that contains only __one__ physical vehicle. A use case for the debug plotter is also given in `firstTest`. The plotter is registered to the simulation as an observer and is then generating one visualisation chart for each loop iteration.
