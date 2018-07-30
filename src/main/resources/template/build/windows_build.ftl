@ECHO OFF

IF EXIST build\ RMDIR /S /Q build\
mkdir build
cd build

<#if usemingw >
cmake .. -G "MinGW Makefiles"
<#else >
cmake ..
</#if>

cmake --build . --target StreamTests