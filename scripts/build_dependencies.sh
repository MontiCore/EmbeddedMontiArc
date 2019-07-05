#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR/..
echo ""
echo "**************************************"
echo "           Making Unicorn"
echo "**************************************"
pushd unicorn
./make.sh
popd

echo ""
echo "**************************************"
echo "      Making Zydis Release/Debug"
echo "**************************************"
pushd zydis
pushd dependencies/zycore
../../../scripts/build.sh Release
../../../scripts/build.sh Debug
popd
../scripts/build.sh Release
../scripts/build.sh Debug
popd

echo ""
echo "**************************************"
echo "    Making pe-parse Release/Debug"
echo "**************************************"
pushd pe-parse
../scripts/build.sh Release
../scripts/build.sh Debug
popd


cp "unicorn/libunicorn.a" "hardware_emulator/libs/Release/"
cp "unicorn/libunicorn.a" "hardware_emulator/libs/Debug/"
cp "zydis/Release/libZydis.a" "hardware_emulator/libs/Release/"
cp "zydis/Debug/libZydis.a" "hardware_emulator/libs/Debug/"

cp "pe-parse/Release/pe-parser-library/libpe-parser-library.a" "hardware_emulator/libs/Release/"
cp "pe-parse/Debug/pe-parser-library/libpe-parser-library.a" "hardware_emulator/libs/Debug/"

popd