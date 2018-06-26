# EMAM2Middleware

## Writing your own Middleware Generator
see [TUTORIAL_ADD_MIDDLEWARE.md](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/TUTORIAL_ADD_MIDDLEWARE.md)

## Usage
### CLI
Maven generates the jar `embedded-montiarc-math-middleware-generator-{Version}-jar-with-dependencies.jar`
and the cli is located in `de.monticore.lang.monticar.generator.middleware.DistributedTargetGeneratorCli`.

CLI Options:
* -m/--models-dir: full path to directory with EMAM models
* -r/--root-model: fully qualified name of the root model
* -o/--output-dir: full path to output directory for generated files
* -g/--generators: identifiers for the generators that should be used
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

