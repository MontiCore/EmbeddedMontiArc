@REM
@REM (c) https://github.com/MontiCore/monticore
@REM



set MINGW_HOME="C:\msys64\mingw64"
set PATH=%MINGW_HOME%\bin;%PATH%
g++ -fpermissive -shared -fPIC  -o "sample_functioncalling.dll"  "base.cpp" 
