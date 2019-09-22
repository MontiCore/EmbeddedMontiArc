<!-- (c) https://github.com/MontiCore/monticore -->
# EMAM2SomeIP

A project that generates a SomeIP-Adapter for EmbeddedMontiArcMath components.

## Install SOME/IP:

Get vsomeip from: https://github.com/GENIVI/vsomeip

Build instruction and further documentation is in the readme on the site, short summary of the install instruction:
- A C++11 enabled compiler like gcc >= 4.8 is needed.
- vsomeip uses CMake as buildsystem.
- vsomeip uses Boost (boost.org), version has to be >= 1.55 and < 1.66:

#### Install SOME/IP on Linux:

Installing boost 1.65 on ubuntu 18.10:
```bash
sudo apt-get install libboost-system1.65-dev libboost-thread1.65-dev libboost-log1.65-dev
```
Build vsomeip from vsomeip main directory:
```bash
mkdir build
cd build
sudo cmake ..
sudo make
sudo make install
```

If build fails, try to apply this fix:

https://github.com/GENIVI/vsomeip/issues/25

Summary of the fix:

Replace "return &sockaddr;" with "return reinterpret_cast<struct sockaddr*>(&sockaddr);"
in the file: implementation/endpoints/include/netlink_connector.hpp

##### Build the documentation:

To build the documentation (after building vsomeip) asciidoc, source-highlight, doxygen and graphviz are needed:
```bash
sudo apt-get install asciidoc source-highlight doxygen graphviz
make doc
```

## Use SOME/IP:

One example which uses SOME/IP can be found in the example folder. Building it with cmake will produce two executables, publisher and subscriber. In this example, the publisher publishes the double 2.5, which will be printed to the standard out of the subscriber . One way to try it is by executing the files in two different consoles, starting with the publisher.

#### Use SOME/IP with docker:

A docker image with installed vsomeip can be found in the container registry of this project. The docker image can also be produced by using the dockerfile in the docker folder.

## Install the project

The SomeIP-Adapter can be installed with maven and java, using the command
```bash
mvn clean install -s settings.xml
```
