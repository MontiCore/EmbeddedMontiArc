#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR/..


echo "Cleaning Unicorn"
pushd unicorn
make clean
popd

echo "Cleaning Zydis"
pushd zydis
rm -r Release
rm -r Debug
pushd dependencies/zycore
rm -r Release
rm -r Debug
popd
popd

echo "Cleaning pe-parse"
pushd pe-parse
rm -r Release
rm -r Debug
popd

echo "Cleaning hardware_emulator"
pushd hardware_emulator
rm -r Release
rm -r Debug
popd

popd