if [ "${PWD##*/}" = "scripts" ] 
then
    cd ..
fi

echo ""
echo "**************************************"
echo "           Making Unicorn"
echo "**************************************"
cd unicorn
./make.sh
cd ..

echo ""
echo "**************************************"
echo "      Making Zydis Release/Debug"
echo "**************************************"
cd zydis
cd dependencies/zycore
../../../scripts/build.sh Release
../../../scripts/build.sh Debug
cd ../..
../scripts/build.sh Release
../scripts/build.sh Debug
cd ..

echo ""
echo "**************************************"
echo "    Making pe-parse Release/Debug"
echo "**************************************"
cd pe-parse
../scripts/build.sh Release
../scripts/build.sh Debug
cd ..


cp "unicorn/libunicorn.a" "hardware_emulator/libs/Release/"
cp "unicorn/libunicorn.a" "hardware_emulator/libs/Debug/"
cp "zydis/Release/libZydis.a" "hardware_emulator/libs/Release/"
cp "zydis/Debug/libZydis.a" "hardware_emulator/libs/Debug/"

cp "pe-parse/Release/pe-parser-library/libpe-parser-library.a" "hardware_emulator/libs/Release/"
cp "pe-parse/Debug/pe-parser-library/libpe-parser-library.a" "hardware_emulator/libs/Debug/"