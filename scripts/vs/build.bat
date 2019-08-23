@rem (c) https://github.com/MontiCore/monticore  

@echo off

@REM -------------------------------------------------------------------------------------------------------------
@REM This script generates the Visual Studio files for a project.
@REM Call from within a CMake project directory.
@REM -------------------------------------------------------------------------------------------------------------


IF [%1] == [] (SET TARGET="Release") else (SET TARGET="%1")

if not exist build mkdir build

echo [SCRIPT] Building with target %TARGET%...
pushd build
cmake -DCMAKE_BUILD_TYPE=%TARGET% -G "Visual Studio 16 2019" ../
popd
