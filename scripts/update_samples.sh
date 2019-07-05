#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR/../samples

echo "**************************************"
echo "        Updating simple sample"
echo "**************************************"
pushd simple
make
cp "sample_simple.so" "../../hardware_emulator/bin/"
popd


echo "**************************************"
echo "     Updating funccalling sample"
echo "**************************************"
pushd funccalling
make
cp "sample_functioncalling.so" "../../hardware_emulator/bin/"
popd

echo "**************************************"
echo "      Updating syscall sample"
echo "**************************************"
pushd syscall_so
make
cp "sample_syscall.so" "../../hardware_emulator/bin/"
popd

popd
