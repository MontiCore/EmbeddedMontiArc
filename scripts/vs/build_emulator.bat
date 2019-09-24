@REM
@REM (c) https://github.com/MontiCore/monticore
@REM
@REM The license generally applicable for this project
@REM can be found under https://github.com/MontiCore/monticore.
@REM



@REM -------------------------------------------------------------------------------------------------------------
@REM This script builds and compiles the hardware_emulator (and installs it to the Maven project)
@REM -------------------------------------------------------------------------------------------------------------

@echo off
set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..
set EMU_DIR=%ROOT_DIR%\hardware_emulator

IF [%1] == [] (SET GENERATOR="Visual Studio 16 2019") else (SET GENERATOR=%1)

pushd %EMU_DIR%
call %SCRIPTS_DIR%\build_compile.bat hardware-emulator Release %GENERATOR%
echo [SCRIPT] Installing Emulator in Maven project...
msbuild -verbosity:quiet build\INSTALL.vcxproj /m /p:Configuration=Release /p:PlatformShortName=x64
popd
