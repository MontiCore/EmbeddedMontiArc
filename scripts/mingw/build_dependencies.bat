@REM
@REM
@REM  ******************************************************************************
@REM  MontiCAR Modeling Family, www.se-rwth.de
@REM  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
@REM  All rights reserved.
@REM
@REM  This project is free software; you can redistribute it and/or
@REM  modify it under the terms of the GNU Lesser General Public
@REM  License as published by the Free Software Foundation; either
@REM  version 3.0 of the License, or (at your option) any later version.
@REM  This library is distributed in the hope that it will be useful,
@REM  but WITHOUT ANY WARRANTY; without even the implied warranty of
@REM  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
@REM  Lesser General Public License for more details.
@REM
@REM  You should have received a copy of the GNU Lesser General Public
@REM  License along with this project. If not, see <http://www.gnu.org/licenses/>.
@REM *******************************************************************************
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