# EMAM2Middleware
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/badges/master/coverage.svg)
## Purpose
This generator takes an EMAM or EMADL model and connects it to a middleware library. If all Ports of two connected Components are marked as middleware Ports, the generator will create 2 executables that can be deployed on different machines.
All communication of these 2 Components will then be tunneled trough the specified middleware:
![MiddlewareAdapter](/uploads/6e9c69e6b56554579551769174df3697/MiddlewareAdapter.png)

## Other important documents
### Quickstart
If you want to use the generator for your project, check out [QUICKSTART_USER.md](QUICKSTART_USER.md).

If you want to add features to this generator, check out [QUICKSTART_DEVELOPER.md](QUICKSTART_DEVELOPER.md)

### Writing your own Middleware Generator
see [TUTORIAL_ADD_MIDDLEWARE.md](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/TUTORIAL_ADD_MIDDLEWARE.md)

### Dependencies needed to compile the generated projects
See [INSTALL_DEPENDENCIES.md](INSTALL_DEPENDENCIES.md)

## Usage
### CLI
Maven generates the jar `embedded-montiarc-math-middleware-generator-{Version}-jar-with-dependencies.jar`
and the cli is located in `de.monticore.lang.monticar.generator.middleware.DistributedTargetGeneratorCli`.

Parameters: `${file path to config json}` OR `-r ${raw json config string}`
```
Schema of config json:
{
    'modelsDir':'<path to directory with EMAM models>',
    'outputDir':'<path to output directory for generated files>',
    'rootModel':'<fully qualified name of the root model>',
    'generators':['<identifier for first generator>', '<identifier for second generator>',...],
    'emadlBackend':'<deep-learning-framework backend. Options: MXNET, CAFFE2>'
}
```
Generator Options:
- Behaviour generators:
    - 'cpp': EMAM2CPP
    - 'emadlcpp': EMADL2CPP
- Middleware generators:
    - 'roscpp': EMAM2Roscpp
    
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

## Running the Integration tests locally
To run the integration tests locally, `docker` needs to be installed. Install instructions can be found [here](https://docs.docker.com/install/).

Run the tests by executing [dockerLocalIntegrationTestRos.sh](src/test/bash/dockerLocalIntegrationTestRos.sh) or [dockerLocalIntegrationTestRos2.sh](src/test/bash/dockerLocalIntegrationTestRos2.sh) as root:
```bash
sudo src/test/bash/dockerLocalIntegrationTestRos.sh
sudo src/test/bash/dockerLocalIntegrationTestRos2.sh
```