<!-- (c) https://github.com/MontiCore/monticore -->
<a href="https://codeclimate.com/github/MontiSim/commons/maintainability"><img src="https://api.codeclimate.com/v1/badges/1fee5e810477b6a2a432/maintainability" /></a>   [![Build Status](https://travis-ci.org/MontiSim/commons.svg?branch=master)](https://travis-ci.org/MontiSim/commons)   [![Coverage Status](https://coveralls.io/repos/github/MontiSim/commons/badge.svg?branch=master)](https://coveralls.io/github/MontiSim/commons?branch=master)

# Commons
This repository includes the interfaces and general functionality for the other repositories.

It should be noted that this repository should not have any dependencies on the other repositories - instead all other repositories are allowed to have this repository as dependency.

## JavaFX replacement

[Point2D](src/main/java/de/rwth/monticore/EmbeddedMontiArc/simulators/commons/utils/Point2D.java), [Point3D](src/main/java/de/rwth/monticore/EmbeddedMontiArc/simulators/commons/utils/Point3D.java) and [Pair](src/main/java/de/rwth/monticore/EmbeddedMontiArc/simulators/commons/utils/Pair.java) replace the classes of the same name from the JavaFX library.

## LibraryService

The [LibraryService](src/main/java/de/rwth/monticore/EmbeddedMontiArc/simulators/commons/utils/LibraryService.java) allows to export native libraries from the resources of a jar at runtime.
