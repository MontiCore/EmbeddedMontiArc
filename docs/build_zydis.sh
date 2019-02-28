cd dependencies/zycore
mkdir build
cd build
cmake -G "Unix Makefiles" ../
make
cd ../../..
mkdir build
cd build
cmake -G "Unix Makefiles" ../
make
cd ..
