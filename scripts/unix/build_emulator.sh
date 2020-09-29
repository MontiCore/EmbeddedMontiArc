#!/bin/bash


SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#Import print()
. $SCRIPTS_DIR/print.sh
ROOT_DIR="$SCRIPTS_DIR/../.."
EMU_DIR="$ROOT_DIR/hardware_emulator"


pushd $EMU_DIR
$SCRIPTS_DIR/build_compile.sh
print "Installing Emulator in Maven project..."
cd build
make install
popd
