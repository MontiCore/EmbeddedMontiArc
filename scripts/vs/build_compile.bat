@REM
@REM (c) https://github.com/MontiCore/monticore
@REM
@REM The license generally applicable for this project
@REM can be found under https://github.com/MontiCore/monticore.
@REM


@REM -------------------------------------------------------------------------------------------------------------
@REM This script generates the Visual Studio project and compiles it.
@REM Call from within a CMake project directory and give SOLUTION NAME as parameter
@REM -------------------------------------------------------------------------------------------------------------

@echo off

set SOLUTION_PATH="%1.sln"
IF [%1] == [] (SET SOLUTION_PATH="hardware-emulator.sln") else (SET SOLUTION_PATH="%1.sln")
IF [%2] == [] (SET TARGET=Release) else (SET TARGET=%2)
IF [%3] == [] (SET GENERATOR="Visual Studio 16 2019") else (SET GENERATOR=%3)

if not exist build mkdir build
cd build
echo [SCRIPT] Building with target %TARGET%...
cmake -G %GENERATOR% ../
echo [SCRIPT] Compiling...
msbuild -verbosity:quiet %SOLUTION_PATH% /m /p:Configuration=%TARGET% /p:PlatformShortName=x64
cd ..
