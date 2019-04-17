# System test

This module contains a simple test to validate the behavior of the simulation.

`MAP_PATH`, `SOURCE_OSM_NODE`, `TARGET_OSM_NODE` and `USE_MODELICA_VEHICLE` in the `Runner` class can be configured to run the simulation with
different settings.

__IMPORTANT__: To run the test, make sure a RMIClient is listening on localhost:10101. For further information please refer
to [RMIModelServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer).