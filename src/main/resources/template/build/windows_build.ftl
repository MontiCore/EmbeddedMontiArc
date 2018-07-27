@ECHO OFF

rmdir /s /q build
mkdir build
cd build
cmake .. > build.log
make StreamTests >> build.log