@REM
@REM (c) https://github.com/MontiCore/monticore
@REM




@REM -------------------------------------------------------------------------------------------------------------
@REM This script removes all the build directories from hardware_emulator and its dependencies
@REM -------------------------------------------------------------------------------------------------------------

@echo off
set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..

echo [SCRIPT] Cleaning all build files...

pushd %ROOT_DIR%

echo [SCRIPT] Cleaning pe-parse...
cd pe-parse
rmdir /S /Q build
cd ..

echo [SCRIPT] Cleaning zydis...
cd zydis
rmdir /S /Q build
cd ..

echo [SCRIPT] Cleaning unicorn...
cd unicorn
rmdir /S /Q build
cd ..

echo [SCRIPT] Cleaning hardware_emulator...
cd hardware_emulator
rmdir /S /Q build
cd ..

popd
