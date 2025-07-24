# autopilot-tools (old vcg-tools)

## Cloning this repository

Include `--recursive` to clone sub-projects. This is the **shared_cpp** project and the  **Armadillo** library (only needed for EMA builds):

```bash
git clone --recursive https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/autopilot-tools
```

## Documents

- [Setup](docs/setup.md)
- [VCG Workflows](docs/vcg-workflows.md): Building, uploading, connecting, running.
- [TCP Protocol](docs/tcp-protocol.md)
- [Network Update](docs/network-update.md)

## Links

- [Simulation Wiki](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/home)
- [basic-simulator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator)

## Cloning and Building projects

```bash
git clone <project_url>
cd <project_folder>
# Normal Build
mvn clean install -s settings.xml
# Build without tests
mvn clean install -s settings.xml -DskipTests
```
