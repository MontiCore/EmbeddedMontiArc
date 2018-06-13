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

REM @echo off
call "..\shared\variables.bat"
setlocal ENABLEDELAYEDEXPANSION

REM set jar= "%NFPVERIFICATION_HOME%\ocl_ema2java-4.0.2-jar-with-dependencies.jar"
if exist "%NFPVERIFICATION_HOME%" rmdir "%NFPVERIFICATION_HOME%" /s /q
mkdir %NFPVERIFICATION_HOME%

set parent_dir= "%HOME%\model\nfpverification"
set target= "%HOME%\model\nfpverification\target"
REM set model= "example.negative.Sensors"
set ocl= "example.rule2"



REM "%JAVA_HOME%\bin\java.exe" -jar %JAR% %parent_dir% %1 %ocl% %target%

REM "%JAVA_HOME%\bin\java.exe" -jar "%VERIFICATION_HOME%\view-verification-0.0.2-SNAPSHOT-jar-with-dependencies.jar" ^
REM 	%HOME%\model\pump\model\pumpStationExample\PumpStation.ema ^
REM 	%target% ^
REM 	%NFPVERIFICATION_HOME%\results


set text=""

for /f "tokens=*" %%a in (%HOME%\model\nfpverification\target\witnesses_example.rule2_%1\__WITNESS_OVERVIEW__.txt) do (
   set text=!text!^<p^>%%a^</p^>
)

set output="%text:"= %"

(echo %output%) >> %NFPVERIFICATION_HOME%\temp.txt 

(
echo ^<!DOCTYPE html^>
echo   ^<html^>^<body^>^<header^>Witness Overview^</header^>^
type  %NFPVERIFICATION_HOME%\temp.txt
echo   ^</body^>^</html^>
) >> %NFPVERIFICATION_HOME%\result.html 

