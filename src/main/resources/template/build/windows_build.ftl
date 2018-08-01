@ECHO OFF

IF EXIST build\ RMDIR /S /Q build\
mkdir build
cd build

<#if genNONE >
cmake ..
<#else>
cmake .. -G "${generator}"
</#if>

cmake --build . --target StreamTests ${buildoptions}