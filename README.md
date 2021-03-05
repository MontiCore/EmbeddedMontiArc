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
  look at CeilMathCommand.java as an example for an command that already exists in Armadillo. For a
  more complex command, you can look at scaleCube. CeilMathCommand.java will be find in adi-dev branch, after merging this branch into master the
  ceil command will be available also on the master.
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

### Note to use CV-Commands:
* Available since version 0.1.16-SNAPSHOT. 
* For conversion between _Armadillo_-matrix/cube and _OpenCV_-matrix/cube it exists a helper-header _ConvHelper.h_. This is generated automatically if at least one CV-command is used.
* For more information about how to use CV-Command, see the _BallTracking_-project in the application repository.

## Install Loop Solver Code Generation

### [Odeint](http://headmyshoulder.github.io/odeint-v2/)
* Download the header only library Boost [here](https://www.boost.org/)
* Extract the library to a destination of your choice (e.g. `C:\lib`)
* Add the environment variable `BOOST_HOME` with the library as value (e.g. `C:\lib\boost`)

### [DAECPP](https://github.com/ikorotkin/dae-cpp)
* Follow the steps described [here](https://github.com/ikorotkin/dae-cpp#installation) to install the 
  [Intel Math Kernel Library](https://software.intel.com/content/www/us/en/develop/tools/oneapi/components/onemkl.html)
  version `2019.3`
* Add the environment variable `MKL_HOME` (e.g. `C:\lib\mkl\compilers_and_libraries_2019.3.203\windows\mkl`)
* Clone the project dae-cpp from git to a destination of your choice (e.g. `C:\lib\install`)
  * `git clone https://github.com/ikorotkin/dae-cpp.git`
* Navigate to this directory and replace the CMakeLists with the one found in `native/daecpp/CMakeLists.txt` 
  (e.g. `C:\lib\install\daecpp\CMakeLists.txt`)
* Create a build subdirectory and navigate to it (e.g. `C:\lib\install\build`)
* Run this cmake command. For `$installDir` enter the destination directory
  * `cmake .. -DCMAKE_INSTALL_PREFIX=$installDir`
  * e.g. `cmake .. -DCMAKE_INSTALL_PREFIX=C:/lib/dae-cpp/`
* Add the environment variable `DAECPP_HOME` with value `$installDir` (e.g. `C:\lib\daecpp`)

### Use DAECPP
* To use this library the MKL variables need to be set first
* Open command line
  * Include the variables
    * Win `"%MKL_HOME%\bin\mklvars.bat" intel64`
    * Linux `source $MKL_HOME/bin/mklvars.sh intel64`
  * In generated code directory:
    * create build dir
  * In build dir:
    * cmake ..
    * cmake --build .  --config release