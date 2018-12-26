cd dependencies/zycore
mkdir build
cd build
cmake -G "CodeBlocks - Unix Makefiles" ../
make
cd ../../..
mkdir build
cd build
cmake -G "CodeBlocks - Unix Makefiles" ../
make
cmake -G "CodeBlocks - Unix Makefiles" -DZYDIS_BUILD_SHARED_LIB=ON ../
make
cd ..
