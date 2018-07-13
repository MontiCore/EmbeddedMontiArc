# Simulation
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/badges/master/coverage.svg)

This repository includes the classes for the management of the simulation, the physics computations, the environment, the sensors and the vehicles.

Discrete time steps are used to advance the the simulation and a rigid body simulation is used for the physics computations.

The environment is imported from OpenStreetMap data.

# Simulation modes
Several simulation modes are available in this module.

An example for the recommended simulation mode is shown in the function `setupSimulator()` in the class `SimulatorMain.java` from the application repository.
