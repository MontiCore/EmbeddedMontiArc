if exist build del /F /Q build
mkdir build
cd build
cmake ..
cmake --build .  --config release
cd ..