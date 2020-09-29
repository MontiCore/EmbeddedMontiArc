@REM
@REM (c) https://github.com/MontiCore/monticore
@REM




@REM -------------------------------------------------------------------------------------------------------------
@REM This script generates the MinGW makefiles and compiles them.
@REM Call from within a CMake project directory
@REM -------------------------------------------------------------------------------------------------------------

@echo off

IF [%1] == [] (SET TARGET="Release") else (SET TARGET="%1")

if not exist build mkdir build
echo [SCRIPT] Building with target %TARGET%...
cd build
cmake -DCMAKE_BUILD_TYPE=%TARGET% -G "MinGW Makefiles" ../
echo [SCRIPT] Compiling...
mingw32-make
cd ..
