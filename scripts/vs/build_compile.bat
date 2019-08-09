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
@REM This script generates the MinGW makefiles and compiles them.
@REM Call from within a CMake project directory and give SOLUTION NAME as parameter
@REM -------------------------------------------------------------------------------------------------------------

@echo off

set SOLUTION_PATH="%1.sln"
IF [%2] == [] (SET TARGET="Release") else (SET TARGET="%2")

if not exist build mkdir build
cd build
echo [SCRIPT] Building with target %TARGET%...
cmake -G "Visual Studio 16 2019" ../
echo [SCRIPT] Compiling...
msbuild -verbosity:quiet %SOLUTION_PATH% /m /p:Configuration=%TARGET% /p:PlatformShortName=x64
cd ..