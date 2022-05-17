set -e
rm -rf build
mkdir build
cd build
cmake ..
make
rm -rf ../target
mkdir ../target
cp libROSInterface.so ../target
