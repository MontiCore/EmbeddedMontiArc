@rem (c) https://github.com/MontiCore/monticore  

setlocal ENABLEDELAYEDEXPANSION
REM @echo off
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
xcopy /s/y %%g "%HOME%\model\nfpverification\target\witnesses_%1_example.model.Sensors\"
)

cd "%HOME%\model\nfpverification\target\witnesses_%1_example.model.Sensors"

for /r . %%g in (*.ema) do (
call "%HOME%\scripts\nfpverification\visualize" %%~nxg %1
)

REM ()>> %NFPVERIFICATION_HOME%\results\result.html




