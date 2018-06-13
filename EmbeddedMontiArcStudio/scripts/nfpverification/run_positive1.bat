@REM
@REM ******************************************************************************
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

@echo off
call "..\shared\variables.bat"

set jar= "%~dp0%ocl_ema2java-4.0.2-jar-with-dependencies.jar"

set parent_dir= "%HOME%\model\nfpverification"
set target= "%HOME%\NFPVerification\target"
set model= "example.positive.Sensors"
set ocl= "example.rule1"



"%JAVA_HOME%\bin\java.exe" -jar %JAR% %parent_dir% %model% %ocl% %target%



