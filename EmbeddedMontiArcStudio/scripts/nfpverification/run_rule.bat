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
setlocal ENABLEDELAYEDEXPANSION
@echo off
call "..\shared\variables.bat"

if exist "%HOME%\nfpverification\results\" rmdir "%HOME%\nfpverification\results\" /s /q
mkdir %HOME%\nfpverification\results\

set jar= "%NFPVERIFICATION_HOME%\ocl_ema2java-4.0.3-SNAPSHOT-jar-with-dependencies.jar"

set parent_dir= "%HOME%\model\nfpverification"
set target= "%HOME%\model\nfpverification\target"
set model= "example.model.Sensors"
REM set ocl= "example.rule1"

"%JAVA_HOME%\bin\java.exe" -jar %JAR% %parent_dir% %model% %1 %target%

if exist "%NFPVERIFICATION_HOME%\results\" rmdir "%NFPVERIFICATION_HOME%\results\" /s /q
mkdir %NFPVERIFICATION_HOME%\results\


cd "%HOME%\model\nfpverification\example"

for /r . %%g in (*.tag) do (
xcopy /s/y %%g "%HOME%\model\nfpverification\target\witnesses_%1_example.negative.Sensors\"
)

cd %HOME%\model\nfpverification\target\witnesses_%1_example.negative.Sensors

for /r . %%g in (*.ema) do (
set file=%%~nxg 
set file=!file:~0,-4!
"%JAVA_HOME%\bin\java.exe" -jar "%SVG_HOME%\embeddedmontiarc-svggenerator.jar" ^
   --input "!file!" ^
   --modelPath "%HOME%\model\nfpverification\target\witnesses_%1_example.negative.Sensors" ^
   --recursiveDrawing "true" ^
   --outputPath "%NFPVERIFICATION_HOME%\results\\" 
)

(
echo ^<^!DOCTYPE html^>^ 
echo ^<^html^>^<^body^>^<^header^>^Witness Overview^<^/header^>^
)>> %NFPVERIFICATION_HOME%\results\result.html




