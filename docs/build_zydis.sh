mkdir build
cd build
cmake -G "Unix Makefiles" ../
make
cmake -G "Unix Makefiles" -DZYDIS_BUILD_SHARED_LIB=ON ../
make
cd ..
