cd unicorn
./make.sh
cd ..
cd zydis
../docs/build_zydis.sh
cd ..
cd pe-parse
mkdir build
cd build
cmake -G "Unix Makefiles" ../
make
cd ../..
cd hardware_emulator
mkdir build
cd build
cmake -G "Unix Makefiles" ../
make
cd ..
