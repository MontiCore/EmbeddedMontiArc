@REM
@REM (c) https://github.com/MontiCore/monticore
@REM
@REM The license generally applicable for this project
@REM can be found under https://github.com/MontiCore/monticore.
@REM


set MINGW_HOME="A:\EmbededMontiArcNew\EMAM-showcase\mingw64"
set PATH=%MINGW_HOME%\bin;%PATH%
g++ -shared -fPIC  -o "sample_functioncalling.dll"  "base.cpp" 
