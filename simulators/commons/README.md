<!-- (c) https://github.com/MontiCore/monticore -->
# Commons

This repository includes the interfaces and general functionality for the other MontiSim projects.
It should be noted that this repository should not have any dependencies on the other repositories - instead all other repositories are allowed to have this repository as dependency.

## >> [Link to the MontiSim Documentation (Wiki)](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/home) <<

The Issues for MontiSim are tracked in the [simulation project](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/boards/8343).

## Usage

To use the different sub-project as maven dependency, add the following to your pom's `<dependencies>`:

```xml
<dependency>
    <groupId>montisim</groupId>
    <artifactId>commons</artifactId>
    <version>${version}</version>
</dependency>
```
