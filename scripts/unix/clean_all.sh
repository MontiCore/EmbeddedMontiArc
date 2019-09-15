#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore.
#

#

SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#Import print()
. $SCRIPTS_DIR/print.sh
ROOT_DIR="$SCRIPTS_DIR/../.."


print "Cleaning all build files..."

pushd $ROOT_DIR

print "Cleaning pe-parse"
pushd pe-parse
rm -r build
popd

print "Cleaning Zydis"
pushd zydis
rm -r build
# pushd dependencies/zycore
# rm -r build
# popd
popd

print "Cleaning Unicorn"
pushd unicorn
make clean
popd

print "Cleaning hardware_emulator"
pushd hardware_emulator
rm -r build
popd

popd
