@REM
@REM (c) https://github.com/MontiCore/monticore
@REM




@REM -------------------------------------------------------------------------------------------------------------
@REM This script builds and compiles the hardware_emulator (and installs it to the Maven project)
@REM -------------------------------------------------------------------------------------------------------------

@echo off
set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..
set EMU_DIR=%ROOT_DIR%\hardware_emulator

pushd %EMU_DIR%
call %SCRIPTS_DIR%\build_compile.bat
echo [SCRIPT] Installing Emulator in Maven project...
cd build
mingw32-make install
popd
