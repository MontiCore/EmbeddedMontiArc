@ECHO OFF

IF EXIST build\ RMDIR /S /Q build\

mkdir build
cd build
cmake .. > build.log
make StreamTests >> build.log