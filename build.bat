if exist build del /F /Q /S build
mkdir build
cd build
cmake ..
cmake --build .  --config release
cd ..