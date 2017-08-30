# Documentation
This documentation includes all relevant information about the implemented concepts and the required installation steps for the MontiSim project.

# Required Software
- Git
- Apache Maven
- Java Development Kit, Version 8
- Java IDE (recommended: IntelliJ IDEA)
- Browser with WebGL support (recommended: Chrome)

# Installation Steps
All parts are located in separate Git repositories, which need to be cloned:

```bash
git clone https://github.com/MontiSim/application.git
git clone https://github.com/MontiSim/commons.git
git clone https://github.com/MontiSim/controller.git
git clone https://github.com/MontiSim/simulation.git
git clone https://github.com/MontiSim/visualization.git
```

For an easier installation the file https://raw.githubusercontent.com/MontiSim/Documentation/master/mainPom/pom.xml needs to be created as `pom.xml` file in that directory which contains all previously downloaded repositories.

After this step it is possible to compile all sub-projects with a single command:

```bash
mvn install
```

There are other useful commands for Maven. This command clears all previously compiled files:

```bash
mvn clean
```

This command installs everything without running all automatic tests, which is faster. This is case sensitive:

```bash
mvn install -DskipTests
```

# Simulation Start: IDE
The `main()` function of the file `SimulatorMain.java` in the `application` repository can be started from the IDE. This creates a minimal setup for the simulation. Console and plotter results are created as outputs and it can be used for debugging.

It should be noted that this class specifies its own simulation scenarios independently from the following simulation start with visualization support.

# Simulation Start: Visualization
Use the following commands to start the simulation with visualization support:

```bash
cd visualization
mvn jetty:run
```

After some seconds of waiting, the visualization is available in your web browser: `http://127.0.0.1:8081/`.

In the top right corner you can click `Start` to start the simulation with the specified simulation scenario from the file `SimulationLoopNotifiableController.java`.

For Chrome there is a useful tip:

If your CPU can not create the simulation results fast enough for the visualization, the visualization will begin to stutter. To avoid that, you can click the `Start` button and afterwards you can minimize your Chrome browser window for some time. This allows the simulation to compute the required results without having to display them immediately. After some waiting you can open the browser window again and the visualization should run smoothly.