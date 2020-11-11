@REM
@REM (c) https://github.com/MontiCore/monticore
@REM

@echo off

if "%1"=="--help" set Help=1
if "%1"=="-h" set Help=1
if "%1"=="/?" set Help=1
if defined Help (
    echo Usage: 
    echo    build_emulator.bat [config] [generator]
    echo Where:
    echo    'config' is Debug or Release. Defaults to "Release"
    echo    'generator' is the cmake generator. Defaults to the default cmake generator.
    exit \B 0
)

@REM -------------------------------------------------------------------------------------------------------------
@REM This script builds and compiles the hardware_emulator (and installs it to the Maven project)
@REM -------------------------------------------------------------------------------------------------------------

set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..
set EMU_DIR=%ROOT_DIR%\hardware_emulator

IF [%1] == [] (SET CONFIG=Release) else (set CONFIG=%1)
IF NOT [%2] == [] (SET GENERATOR=%2)

if [%GENERATOR%] == [] (
    echo [SCRIPT] Building the hardware_emulator in %CONFIG% mode with default generator.
) else (
    echo [SCRIPT] Building the hardware_emulator in %CONFIG% mode with generator: %GENERATOR%.
)

pushd %EMU_DIR%
if [%GENERATOR%] == [] (
    cmake -S . -B build
) else (
    cmake -G"%GENERATOR%" -S . -B build
)
echo [SCRIPT] Compiling...
cmake --build build --config %CONFIG%
echo [SCRIPT] Installing...
cmake --install build --config %CONFIG%
popd