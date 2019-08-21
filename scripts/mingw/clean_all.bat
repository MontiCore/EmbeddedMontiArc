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