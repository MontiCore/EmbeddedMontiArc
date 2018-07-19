![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Cpp/badges/master/coverage.svg)
[![PPTX-Docu](https://img.shields.io/badge/PPTX--Docu-2018--05--22-brightgreen.svg)](https://github.com/EmbeddedMontiArc/Documentation/blob/master/reposlides/18.05.22.Docu.EMAM2CPP.pdf)

# EMAM2Cpp

##CMake Generation:
* Available since version 0.0.22-SNAPSHOT.
* If `isGenerateCMakeEnabled()` additionally to the C++ files _CMakeLists.txt_ is generated. 
  This CMake file builds a static library out of the generated components.
* Find package (https://cmake.org/cmake/help/v3.8/command/find_package.html?highlight=i) can be configured via public method `getCMakeConfig()` from the generator class. FindModule files are generated automatically which searches for header include directories and libraries at default locations.    
* Example: `getCMakeConfig().addModuleDependency(new CMakeFindModule("LibName", "LibHeader.hpp", "libname", headerSearchPaths, bibrarySearchPaths, findHeaderEnabled, findLibEnabled, isRequiered));`