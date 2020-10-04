#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#

# -------------------------------------------------------------------------------------------------------------
# This script builds and compiles the hardware_emulator (and installs it to the Maven project)
# -------------------------------------------------------------------------------------------------------------



SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#Import print()
. $SCRIPTS_DIR/print.sh
ROOT_DIR="$SCRIPTS_DIR/../.."
EMU_DIR="$ROOT_DIR/hardware_emulator"

if [ -z "$1" ]; then 
	CONFIG=Release; 
else 
	CONFIG=$1; 
fi

if ! [ -z "$2" ]; then 
	GENERATOR="$2";
fi

pushd $EMU_DIR
#mkdir $build_dir
#cd $build_dir
if [ -z $GENERATOR ]; then 
    print "Building the hardware_emulator in $CONFIG mode. (With default generator.)"
    cmake -DCMAKE_BUILD_TYPE=$CONFIG -S . -B $CONFIG
else 
    print "Building the hardware_emulator in $CONFIG mode with generator: $GENERATOR."
    cmake -DCMAKE_BUILD_TYPE=$CONFIG -G$GENERATOR -S . -B $CONFIG
fi

cmake --build $CONFIG
cmake --install $CONFIG
popd

# UNICORN:
# unicorn/make.sh
# cp "unicorn/libunicorn.a" "$LIBS_DIR"