<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp/badges/master/coverage.svg)
[![PPTX-Docu](https://img.shields.io/badge/PPTX--Docu-2018--05--22-brightgreen.svg)](https://github.com/EmbeddedMontiArc/Documentation/blob/master/reposlides/18.05.22.Docu.EMAM2CPP.pdf)

# EMAM2Cpp

## CMake Generation:
* Available since version 0.0.22-SNAPSHOT. 

### Interface in Java Code
* If `isGenerateCMakeEnabled()` additionally to the C++ files _CMakeLists.txt_ is generated. 
  This CMake file builds a static library out of the generated components.
* Find package (https://cmake.org/cmake/help/v3.8/command/find_package.html?highlight=i) can be configured via public method `getCMakeConfig()` from the generator class. FindModule files are generated automatically which searches for header include directories and libraries at default locations.    
* Example: `getCMakeConfig().addModuleDependency(new CMakeFindModule("LibName", "LibHeader.hpp", "libname", headerSearchPaths, bibrarySearchPaths, findHeaderEnabled, findLibEnabled, isRequiered));`
* Additionally any CMake command can be inserted via `getCMakeConfig().addCMakeCommand("CMAKE_CXX_FLAGS  \"${CMAKE_CXX_FLAGS} -Wno-deprecated\"")`or at the end via `addCMakeCommandEnd("#some command at the end")`

### How to add a new Command to EMAM
* add your command java file in src /src/main/java/de/monticore/lang/generator/cpp/commands. You can
  look at ceilMathCommand.java as an example for an command that already exists in Armadillo. For a
  more complex command, you can look at scaleCube.
* register your command in MathCommandRegisterCPP.java
* add yourCommandTest.emam in EMAM2Cpp/src/test/resources/test/math
* add a cpp file for your command in EMAM2Cpp/src/test/resources/results/armadillo/testMath/l0. This
  file will be compared to the generated one from yourCommandTest.emam.
* add your test in
  EMAM2Cpp/src/test/java/de/monticore/lang/monticar/generator/cpp/armadillo/ArmadilloFunctionTest.java

### Note on find_package
If no search directory is specified CMake will search on default locations. For linux this is _/usr/lib_ , _usr/local/lib_ , _usr/include_ etc. Windows systems does not have a default library path. The generated CMake files also are using environment variables as hint. If a package could not be found but it is installed somewhere on the system please create an environment variable **PackageName_HOME**.  

Here an example for Armadillo:
Create a environment variable called _Armadillo_Home_ with the path to the base directory of your Armadillo installation.

