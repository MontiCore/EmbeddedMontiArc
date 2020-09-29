# Simulation

## >> [Link to the Documentation for this project (Wiki)](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/home)  <<

## Usage

To use the different sub-project as maven dependency, add the following to your pom's `<dependencies>`:

```xml
<dependency>
    <groupId>montisim</groupId>
    <artifactId>*sub-project*</artifactId>
    <version>${revision}</version>
</dependency>
```

`*sub-project*` can be any of the following:

- `environment`
- `eesimulator`
- `vehicle`
- `eecomponents`
- `simulator`
