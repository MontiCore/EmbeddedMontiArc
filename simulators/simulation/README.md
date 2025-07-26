<!-- (c) https://github.com/MontiCore/monticore -->

# Simulation

## > > [Link to the MontiSim Documentation (Wiki)](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/home)  <<

The Issues for MontiSim are tracked in
the [simulation project](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/boards/8343).

## Usage

To use the different sub-project as maven dependency, add the following to your pom's `<dependencies>`:

```xml
<dependency>
    <groupId>montisim</groupId>
    <artifactId>*sub-project*</artifactId>
    <version>${version}</version>
</dependency>
```

`*sub-project*` can be any of the following:

- `environment`
- `eesimulator`
- `vehicle`
- `eecomponents`
- `simulator`
