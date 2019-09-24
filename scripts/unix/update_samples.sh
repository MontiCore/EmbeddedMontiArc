#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore.
#

# (c) https://github.com/MontiCore/monticore  
#

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#Import print()
. $SCRIPTS_DIR/print.sh
ROOT_DIR="$SCRIPTS_DIR/../.."

pushd $ROOT_DIR/samples

print "Updating simple sample..."
pushd simple
make
print "Copying simple sample..."
cp "sample_simple.so" "../../hardware_emulator/bin/"
popd


print "Updating funccalling sample..."
pushd funccalling
make
print "Copying funccalling sample..."
cp "sample_functioncalling.so" "../../hardware_emulator/bin/"
popd

print "Updating syscall sample..."
pushd syscall_so
make
print "Copying syscall sample..."
cp "sample_syscall.so" "../../hardware_emulator/bin/"
popd

popd
