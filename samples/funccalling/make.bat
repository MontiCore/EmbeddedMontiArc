set MINGW_HOME="A:\EmbededMontiArcNew\EMAM-showcase\mingw64"
set PATH=%MINGW_HOME%\bin;%PATH%
g++ -shared -fPIC  -o "sample_functioncalling.dll"  "base.cpp" 