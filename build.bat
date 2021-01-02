@ECHO OFF
if exist build del /F /Q /S build > nul
mkdir build
cd build
@ECHO ON
cmake ..
cmake --build .  --config release
cd ..