# Autopilot CI Showcase

The goal of this project is to showcase and test the development of *Continuous Integration* for autopilots.

The goals of the CI are:

- Testing the compilation of the autopilot code (using EMA code generation and compilation).
- "Unit Testing" the autopilot code using the EMA *stream-tests*.
- Testing the performance of the autopilot by running *simulation scenarios*.

## (Dream) Usage

> **Note:** The following is the *goal* behavior. There is still some missing work to do so that it behaves in the described way.

The goal is to use maven plugins, namely the [EMADL plugin](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/emadl-maven-plugin) and the [Stream-test plugin](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/utilities/maven-streamtest). By adding the plugins to the `pom.xml` in this project, the autopilot developer just has to run `mvn emadl:simulate -s settings.xml` to build and test the autopilot. The CI performs the same maven call to test the autopilot.

## Current Usage

(Temporary?) The autopilot can be built by using a local clone of the *armadillo* library.
Make sure to clone the project recursively for this:

```bash
git clone --recurse-submodules <repo_url>
# If you already cloned the project without --recurse-submodules
git submodule update --init --recursive
```

To build and simulate the autopilot, follow the commands in the `.gitlab-ci.yml`.

```bash
mvn streamtest:streamtest-generator -s settings.xml
temp-build.sh
mvn emadl:simulate -s settings.xml
```

## State of development

The Autopilot CI is **Work in progress**.
Here are the points to be improved:

- There is some Armadillo trickery in the project right now. This is to be able to compile the autopilot without having armadillo installed on the system. However the autopilot compilation has to be done manually (see `.gitlab-ci.yml`).
  - If this project is only to be run in the CI with images that have the proper armadillo setup, the `<importArmadillo>` option in the pom can be removed and the CI script can be simplified to use `mvn streamtest:streamtest-build -s settings.xml`.
- Handle the stages better: right now the different stages (building and simulation) must be run separately. Should they use the maven stage system somehow to be executed only from on command. (Ex: `mvn emadl:simulate -s settings.xml` also builds the autopilot)
- Only use one plugin? Right now multiple plugins must be specified in the pom.
- Handle Failures better: On simulation failure, we don't know why the autopilot failed.
  - To do in MontiSim: describe the failure of different `Tasks` and `Goals`. [Related Issue](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/issues/36)
- Handle Exception logging better.
  - This is general MontiSim development: have exceptions and errors be clearer (context, types, ...) and be better handled. [Related Issue](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/issues/24)
- Handle simulation logging.
  - Also general MontiSim development: use logging consistently over the whole simulator. [Related Ticket](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/issues/25)

## Autopilot

The autopilot used in this project was developed during the practical course "Model-driven Software Engineering for Connected Vehicles".

Its original documentation can be found in the `Documentation` folder:

- [Requirements](Documentation/Requirements.md)
- [Architecture](Documentation/Architecture.md)
- [Car-to-Car Communication Protocol](Documentation/C2C-Protocol.md)
- [Use Cases](Documentation/Scenarios-UseCases.pdf) 
- [Test Cases](Documentation/TestCases.pdf) 
- [Algorithm for trajectory following](Documentation/TrajCalAlgorithm.pdf)
- [Control-Unit tuning](Documentation/Control_unit.pdf)
