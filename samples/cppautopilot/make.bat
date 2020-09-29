

set MINGW_HOME="C:\msys64\mingw64"
set PATH=%MINGW_HOME%\bin;%PATH%
g++ -shared -fPIC -std=c++11 -o "cppautopilot.dll" "dynamic_interface.cpp" "autopilot.cpp"
