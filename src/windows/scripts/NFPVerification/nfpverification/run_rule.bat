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
REM @echo off
call "..\..\common\variables"

if exist "%HOME%\nfpverification\results\" rmdir "%HOME%\nfpverification\results\" /s /q
mkdir %HOME%\nfpverification\results\

set jar= "%NFPVERIFICATION_HOME%\ocl_ema2java-4.0.3-SNAPSHOT-jar-with-dependencies.jar"

set parent_dir= "%HOME%\models\NFPVerification"
set target= "%HOME%\models\NFPVerification\target"
set model= "example.model.Sensors"
REM set ocl= "example.rule1"

"%JAVA_HOME%\bin\java.exe" -jar %JAR% %parent_dir% %model% %1 %target%

if exist "%NFPVERIFICATION_HOME%\results\" rmdir "%NFPVERIFICATION_HOME%\results\" /s /q
mkdir %NFPVERIFICATION_HOME%\results\


cd "%HOME%\models\NFPVerification\example"

for /r . %%g in (*.tag) do (
xcopy /s/y %%g "%HOME%\models\NFPVerification\target\witnesses_%1_example.model.Sensors\"
)

cd "%HOME%\models\NFPVerification\target\witnesses_%1_example.model.Sensors"

for /r . %%g in (*.ema) do (
call "%HOME%\scripts\NFPVerification\executing\visualize" %%~nxg %1
)

PAUSE
REM ()>> %NFPVERIFICATION_HOME%\results\result.html




