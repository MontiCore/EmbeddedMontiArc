<!-- (c) https://github.com/MontiCore/monticore -->


# Basic Simulator

This project is a minimal setup running the MontiSim simulator in a local self-contained jar.

### Quickstart

If you know about MontiSim and Maven, the following will suffice you:

> ```
> git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator
> cd basic-simulator
> mvn clean install -s settings.xml
> ```
> The resulting jar (`target/basic-simulator-1.*.jar`) can be launched normally in any folder and will create
> the required sub-folders and export the needed libraries.

> The `build_install` script build the project and puts the jar in the `install` folder, which contains a sample setup.

If not, the following documents aim at giving an introduction to project building, Maven, scripts and MontiSim.

### Documentation Contents

- [Installing and using the Basic Simulator](docs/usage.md)
- [Maven Project tutorial](docs/maven.md)
- [Scripts tutorial](docs/scripts.md)
- [MontiSim overview](docs/montisim.md)

---
