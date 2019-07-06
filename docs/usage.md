Next: [Maven Project tutorial](maven.md)

---

# Installing and using the Basic Simulator



## Installation

This repository is a [Maven](https://maven.apache.org/) project.

In a folder (we recommend making a "MontiSim" folder containing all the cloned projects), clone the repository using the following in the console:
```batch
git clone https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator
```
This will create a folder named `basic-simulator` and download the contents of this repository inside.

> **NOTE**: To open a console in a specific working directory rapidly in windows, type "cmd" in the navigation bar of the windows explorer and press *Enter*.
> Choose *Right Clic -> Open in terminal* on the folder under Linux.
>
> ![Select the navigation bar.](images/cmd1.png "Select the navigation bar.")
>
> ![Type in "cmd" then press Enter.](images/cmd2.png "Type in cmd then press Enter.")

To compile the Maven project, run the `install` script (*.bat* under Windows and *.sh* under Unix).

What this script does is running this command: 
```batch
mvn clean install -s settings.xml
```
This will execute the target "install" of the maven project which will compile, test and create a jar of the project. 
It will then put the jar in the `install` directory of this project as well as in the **local maven repository**. 
More on Maven in the [Maven Project tutorial](docs/maven.md) section. 

**NOTE**: Don't forget the `-s settings.xml` option. This tells Maven where the dependencies of the MontiSim project are located 
(_in the **Nexus** of the Software Engineering Chair_).

The `install` folder contains a script to start the simulator as well as a sample map, scenario and autopilot that can be 
executed.

## Using the Basic Simulator

The compiled jar file can be executed with this java command:
```batch
java -jar basic-simulator.jar
```

This will open a window listing the available **autopilots**, **maps**, **scenarios** and simulation **results**. These are 
located alongside the `basic-simulator.jar` file inside folders with the same names.

By selecting a **scenario**, you can press the "Start Simulation" button to run this scenario in the simulator.

> ![Scenario select.](images/usage.png "Select a scenario and press the button.")
> 
> **NOTE**: *Currently the output of the simulation is only shown in the console.*

You can start a simulation directly without interacting with the GUI by passing a scenario name as command line argument
to the simulator:
```batch
java -jar basic-simulator.jar straight
```
> - *This would start a simulation directly with the scenario "straight"*
> - *This can be used in a script as well*

## Scenario format

The Basic Simulator uses **JSON** scenarios. This is for easier prototyping and parsing of the scenarios. The scenario [straight](../install/scenarios/straight.json) is a good
example of how to configure a scenario. For the complete configuration entries currently supported, look at the entries in the 
[BasicController.ScenarioSettings](../src/main/java/de/rwth_aachen/se/montisim/simulators/basic_simulator/controller/BasicController.java#L35) enum.
The configuration specific to vehicles can be seen in 
[VehicleBuilder.VehicleSettings](../src/main/java/de/rwth_aachen/se/montisim/simulators/basic_simulator/controller/VehicleBuilder.java#L72).

## Working on Maven projects

- To easily work on maven projects, we recommend using the [IntelliJ](https://www.jetbrains.com/idea/) editor.
- To automatically import a Maven project inside IntelliJ, open the **pom.xml** file with IntelliJ (*Open with... -> IntelliJ*).
- You can enable "**Auto import**" in the IntelliJ settings (*File -> Settings -> Build, Execution, Deployment -> Build Tools -> Maven -> Import Maven projects automatically*).
  This will try to update the project when the *pom.xml* file is changed or Maven has downloaded new dependencies.

### Compiling

Compiling might not work in IntelliJ natively because of the complex dependency setup of the simulator. Because of this compile the project using
the Maven command (*mvn clean install ...*), the easiest is to have it in a script. (For the basic-simulator, the `build_install` script will do just that.)

To avoid running all the tests *when compiling for a small change* (**always run the tests before commiting/pushing**), add the **-DskipTests** argument to the 
maven command. (`mvn clean install -s settings.xml -DskipTests`)

## Debugging the simulator

This setup of the MontiSim simulator is local and self-contained. This allows you to run the java debugger on the jar.

For this create a Run/Debug configuration that executes the jar and specify a folder in which to run it. (Ex: the *install* folder inside the basic-simulator project).
You can then start the simulator in normal or debug mode, set breakpoints and step through almost the entire code. 

(**NOTE**: To step through the [Hardware Emulator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/hardware_emulator) you need to attach a C++
debugger to the Java simulator process.)

> Example IntelliJ configuration for running the jar in the **install** folder (Automatically exported with the `build_install` script):
>
> ![Config1](images/config1.png "Select Edit configurations.")
>
> ![Config2](images/config2.png "Create a new JAR config.")
>
> ![Config3](images/config3.png "Select the jar and the working directory.")

---
Next: [Maven Project tutorial](maven.md)