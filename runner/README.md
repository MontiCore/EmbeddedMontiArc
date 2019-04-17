# System test

This module contains a simple test to validate the behavior of the simulation.

`mapPath`, `sourceOsmNode`, `targetOsmNode` and `useModelicaVehicle` can be configured as `Runner` is created to run different tests.
different settings.

Uncomment the code in *test/java/simulation/api/ControllerTest.java* to run the tests.

__IMPORTANT__: To run the test, make sure a RMIClient is listening on localhost:10101. For further information please refer
to [RMIModelServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer).
