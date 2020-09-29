@REM
@REM (c) https://github.com/MontiCore/monticore
@REM



set MINGW_HOME="C:\msys64\mingw64"
set PATH=%MINGW_HOME%\bin;%PATH%
g++ -shared -fPIC -std=c++11 -static-libstdc++ -O3 -o "cppautopilotzigzag.dll" "dynamic_interface.cpp" "autopilot.cpp" "json.cpp" "printf.c"
