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
@REM This script re-compiles all the sample programs used in the hardware_emulator tests.
@REM -------------------------------------------------------------------------------------------------------------

@echo off
pushd %~dp0\..\..
cd samples

echo [SCRIPT] Updating simple sample...
cd simple
call make.bat
echo [SCRIPT] Copying simple sample...
move "sample_simple.dll" "..\..\hardware_emulator\bin\sample_simple.dll"
cd ..

echo [SCRIPT] Updating funccalling sample...
cd funccalling
call make.bat
echo [SCRIPT] Copying funccalling sample...
move "sample_functioncalling.dll" "..\..\hardware_emulator\bin\sample_functioncalling.dll"
cd ..

echo [SCRIPT] Updating syscall sample...
cd syscall_dll
call make.bat
echo [SCRIPT] Copying syscall sample...
move "sample_syscall.dll" "..\..\hardware_emulator\bin\sample_syscall.dll"
cd ..
