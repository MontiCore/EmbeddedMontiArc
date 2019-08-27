<!-- (c) https://github.com/MontiCore/monticore -->
# System test

This module contains a simple test to validate the behavior of the simulation.

`mapPath`, `sourceOsmNode`, `targetOsmNode` and `useModelicaVehicle`, `autopilotHost` and `autopilotPort` can be
configured while constructing the `Runner` to run the test with different settings.

To run the test with docker:

```bash
$ ./run_test
```

Or with docker-compose:
```bash
$ docker-compose up runner
```

To run it without docker: start a RMIServer in the background, set the `autopilotHost` and `autopilotPort`
in _runner/src/test/java/simulation/runner/RunnerTest.java_ accordingly, and then:
```bash
mvn -f ../pom.xml -s ../settings.xml clean install -DskipTests && mvn -s ../settings.xml -Dtests=RunnerTest test
```
 
For further information about RMIModelServer please refer
to [RMIModelServer](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/RMIModelServer).
