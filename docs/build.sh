echo
echo **************************************
echo            Making Unicorn
echo **************************************
cd unicorn
./make.sh
cd ..

echo
echo **************************************
echo              Making Zydis
echo **************************************
cd zydis
../docs/build_zydis.sh
cd ..

echo
echo **************************************
echo        Making pe-parse Release
echo **************************************
cd pe-parse
mkdir Release
cd Release
cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ../
make
cd ..

echo
echo **************************************
echo         Making pe-parse Debug
echo **************************************
mkdir Debug
cd Debug
cmake -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" ../
make
cd ../..

echo
echo **************************************
echo    Making hardware_emulator Release
echo **************************************
cd hardware_emulator
mkdir Release
cd Release
cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ../
make
cd ..

echo
echo **************************************
echo     Making hardware_emulator Debug
echo **************************************
mkdir Debug
cd Debug
cmake -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" ../
make
cd ../..
