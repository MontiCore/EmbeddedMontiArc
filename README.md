# EMAM2Middleware
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/badges/master/coverage.svg)
## Purpose
This generator takes an EMAM or EMADL model and connects it to a middleware library. If all Ports of two connected Components are marked as middleware Ports, the generator will create 2 executables that can be deployed on different machines.
All communication of these 2 Components will then be tunneled trough the specified middleware:
![MiddlewareAdapter](/uploads/6e9c69e6b56554579551769174df3697/MiddlewareAdapter.png)


## Writing your own Middleware Generator
see [TUTORIAL_ADD_MIDDLEWARE.md](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/TUTORIAL_ADD_MIDDLEWARE.md)

## Dependencies needed to compile the generated projects
### All generated projects
CMake, Make,a C++ compiler, and Armadillo are required to compile the generated projects.
#### Linux
Gcc is recommended as the C++ compiler.
Example install of all needed packages for ubuntu:
```bash
sudo apt install gcc cmake make
```
Then download Armadillo from here: [Linux](https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F18.10.24-armadillo-linux&files=armadillo-8.500.1-linux.zip) and set the environment variable `Armadillo_HOME` to the base dir of your installation.

To check everything is installed correctly use whereis/ls:
```bash
$ whereis g++
g++: /usr/bin/g++
$ whereis cmake
cmake: /usr/bin/cmake
$ whereis make
make: /usr/bin/make
$ ls "$Armadillo_HOME/include"
armadillo_bits armadillo.h
```

Compiling a generated project:
```bash
cd /path/to/build/directory
cmake /path/to/generated/project/source
make
```

#### Windows
Mingw64 gcc is recommended as the C++ compiler. Download a version with all needed tools from [here](https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fwin64&files=mingw64.zip).

CMake for Windows: https://cmake.org/download/

Make for Windows: http://gnuwin32.sourceforge.net/packages/make.htm

Add the bin directories of all 3 to your PATH variable.

Then download Armadillo from here: [Windows](https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fwin64&files=armadillo-8.200.2.zip) and set the environment variable `Armadillo_HOME` to the base dir of your installation.
To check everything is installed correctly use where/dir:
```batch
> where g++
C:\mingw64\bin\g++.exe
> where cmake
C:\Program Files\CMake\bin\cmake.exe
> where make
C:\Program Files\make-3.81-bin\bin\make.exe
> dir /b "%Armadillo_HOME%\include"
armadillo
armadillo.h
...
```

Compiling a generated project:
```batch
cd C:\path\to\build\directory
cmake C:\path\to\generated\project\source -G "MinGW Makefiles"
make
```

Please note: It is highly recommended, you stick to the exact versions as stated above. Otherwise you might run into trouble regarding the interplay between cmake/make and the Armadillo library. In particular problems have been reported using Cygwin.

### Projects with roscpp generator
Only for generated projects that contain a ROS adapter(e.g. -g=cpp,roscpp).
ROS Kinetic currently only supports Linux and the installation is described [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Usage
### CLI
Maven generates the jar `embedded-montiarc-math-middleware-generator-{Version}-jar-with-dependencies.jar`
and the cli is located in `de.monticore.lang.monticar.generator.middleware.DistributedTargetGeneratorCli`.

CLI Options:
* -m/--models-dir: full path to directory with EMAM models
* -r/--root-model: fully qualified name of the root model
* -o/--output-dir: full path to output directory for generated files
* -g/--generators: identifiers for the generators that should be used
    * currently supported:
        * cpp
        * roscpp
    * seperated by ','
    * example: cpp,roscpp
    
Example: [CliUsage.sh](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/CliUsage.sh)

### Defining the connection between a component and the middleware
The connection between middleware and the component is defined as tags on Ports in .tag files.
### Example with ROS Middleware:
Tags of the type RosConnection can either be simple tags(see Example 3) or define a topic(http://wiki.ros.org/Topics) with name, type and optional msgField(http://wiki.ros.org/msg , 2.)
Examples:
1. [src/test/resources/tests/a/Add.tag](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/a/Add.tag)
1. [src/test/resources/tests/a/Echo.tag](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/a/Echo.tag)
1. [src/test/resources/tests/dist/SimpleDist.tag](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/dist/SimpleDist.tag)

#### Use-case 1: Creating 1 executable
Look at [GenerationTest::testMiddlewareGenerator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/java/de/monticore/lang/monticar/generator/middleware/GenerationTest.java). The component is defined in [src/test/resources/tests/a/AddComp.emam](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/a/AddComp.emam) and the tags for the connection to ros are defined in [src/test/resources/tests/a/Add.tag](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/a/Add.tag)

#### Use-case 2: Creating multiple executables for distributed systems
Look at [GenerationTest::testDistributedTargetGenerator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/java/de/monticore/lang/monticar/generator/middleware/GenerationTest.java). The component is defined in [src/test/resources/dist/DistComp.emam](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/dist/DistComp.emam) and the tags for the connection to ros are defined in [src/test/resources/tests/dist/SimpleDist.tag](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/src/test/resources/tests/dist/SimpleDist.tag)

#### Compile and run the generated ROS Projects
1. install needed software:
    * ROS Kinetic(http://wiki.ros.org/kinetic/Installation)
    * CMake(https://cmake.org/)
    * Armadillo 8 or higher( www.arma.sourceforge.net)
        * creating a copy of the library named armadillo.h might be necessary.
1. source your ros environment(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment , 2.)
1. a) run src/test/resources/TargetCompilation.sh from **this project's** root
1. or b) compile a single project by
    * changing to the **generated project's** root
    * create a folder build/ and change into it
    * run: cmake ../src
    * run: make
1. Start ros and the other nodes
    * minimal working example: run: roscore
1. If the project was created by a MiddlewareGenerator, run the executable(s) at build/coordinator(/<subcomp.name>)/Coordinator_<(sub)component.name>
2.

## Automatic Clustering
To simplify the creation of distributed systems, the generator can automatically split the model into a given number of clusters.

Supported:
* Spectral Clustering
* ...

Procedure:
* Convert the Symbol Table of a Component into a adjacency matrix
* (Add costs given by type to matrix. E.g. float is cheaper than Matrix of floats)
* Feed into ml library(e.g. [smile ml](https://github.com/haifengl/smile)) with the selected clustering algorithm
* (Compare the result of different algorithms)
* Generate Middleware tags seperating the clusters
* Feed into existing manual clustering architecture
* (generate)



