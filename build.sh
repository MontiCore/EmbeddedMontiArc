rm -rf build
mkdir build && cd build
cmake ..
cmake --build .  --config release
cd ..