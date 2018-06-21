# EMAM2Middleware

## Usage
### Defining the connection between a component and the middleware
The connection between middleware and the component is defined as tags on Ports in .tag files.
#### ROS:
Tags of the type RosConnection can either be simple tags(see Example 3) or define a topic(http://wiki.ros.org/Topics) with name, type and optional msgField(http://wiki.ros.org/msg , 2.)
Examples:
1. src/test/resources/tests/a/Add.tag (TODO: link)
1. src/test/resources/tests/a/Echo.tag (TODO: link)
1. src/test/resources/tests/dist/SimpleDist.tag (TODO: link)

### Use case 1: Creating 1 executable
Look at GenerationTest::testMiddlewareGenerator. The component is defined in src/test/resources/tests/a/AddComp.emam (TODO: link) and the tags for the connection to ros are defined in src/test/resources/tests/a/Add.tag (TODO: link)

### Use case 2: Creating multiple executables for distributed systems
Look at GenerationTest::testDistributedTargetGenerator. The component is defined in src/test/resources/dist/DistComp.emam (TODO: link) and the tags for the connection to ros are defined in src/test/resources/dist/SimpleDist.tag (TODO: link)

### Compile and run the generated Projects
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
