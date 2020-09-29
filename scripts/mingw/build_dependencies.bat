@REM
@REM (c) https://github.com/MontiCore/monticore
@REM




@REM -------------------------------------------------------------------------------------------------------------
@REM This script compiles all the libraries required by the hardware_emulator and places them in the 'libs' folder
@REM -------------------------------------------------------------------------------------------------------------

@echo off
set SCRIPTS_DIR=%~dp0
set ROOT_DIR=%~dp0\..\..
set LIBS_DIR=%ROOT_DIR%\hardware_emulator\libs\mingw
if not exist %LIBS_DIR% mkdir %LIBS_DIR%

IF [%1] == [] (SET DEP_TARGET=Release) else (SET DEP_TARGET=%1)

pushd %ROOT_DIR%

echo.
echo [SCRIPT] Building dependency: pe-parse...
pushd pe-parse
call %SCRIPTS_DIR%\build_compile.bat %DEP_TARGET%
echo [SCRIPT] Copying pe-parse library...
copy "build\pe-parser-library\libpe-parser-library.a" "%LIBS_DIR%\libpe-parser-library.a"
popd

echo.
echo [SCRIPT] Building dependency: Zydis...
pushd zydis
call %SCRIPTS_DIR%\build_compile.bat %DEP_TARGET%
echo [SCRIPT] Copying Zydis library...
copy "build\libZydis.a" "%LIBS_DIR%\libZydis.a"
popd

@REM -------------------------------------------------------------------------------------------------------------
@REM Build Unicorn using MSYS and the default unicorn/make.sh script
@REM Then copy unicorn.a to "%LIBS_DIR%\libunicorn.a"
@REM -------------------------------------------------------------------------------------------------------------

popd
