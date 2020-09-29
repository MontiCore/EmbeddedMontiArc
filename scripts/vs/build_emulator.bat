@REM
@REM (c) https://github.com/MontiCore/monticore
@REM

@echo off

if "%1"=="--help" set Help=1
if "%1"=="-h" set Help=1
if "%1"=="/?" set Help=1
if defined Help (
    echo "Usage: build_emulator.bat [Config (Debug/Release, defaults to Release)] [Generator (defaults to cmake default generator)]"
    exit
)

@REM -------------------------------------------------------------------------------------------------------------
@REM This script builds and compiles the hardware_emulator (and installs it to the Maven project)
@REM -------------------------------------------------------------------------------------------------------------

set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..
set EMU_DIR=%ROOT_DIR%\hardware_emulator

IF [%1] == [] (SET CONFIG=Release) else (set CONFIG=%1)
IF NOT [%2] == [] (SET GENERATOR=%2)

pushd %EMU_DIR%
IF [%GENERATOR%] == [] (
    echo [SCRIPT] Building the hardware_emulator in %CONFIG% mode. (With default generator.)
    cmake -S . -B build
) else (
    echo [SCRIPT] Building the hardware_emulator in %CONFIG% mode with generator: %GENERATOR%.
    cmake -S . -B build
)
cmake --build build --config %CONFIG%
cmake --install build --config %CONFIG%
popd