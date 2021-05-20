@REM
@REM (c) https://github.com/MontiCore/monticore
@REM


set SHARED_CPP_DIR=..\..\extern\shared_cpp
set MINGW_HOME="C:\msys64\mingw64"
set PATH=%MINGW_HOME%\bin;%PATH%
g++ -shared -fPIC -std=c++11 -static-libstdc++ -O3 -I%SHARED_CPP_DIR% -o "cppautopilot.dll" "library_interface.cpp" "autopilot.cpp" "%SHARED_CPP_DIR%\buffer.cpp" "%SHARED_CPP_DIR%\json.cpp" "%SHARED_CPP_DIR%\printf.cpp"
