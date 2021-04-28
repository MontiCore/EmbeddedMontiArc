#!/bin/bash

if [ -z "$1" ] || [ "$1" == "--help" ]; then 
    echo "Usage:"
    echo "    ./ema_build_local.sh <folder>"
    echo "Description:"
    echo "    Builds the EMA CMake project inside the specified folder."
    echo "    Local compile = for this machine."
    exit 0
fi


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $SCRIPT_DIR

. get_config.sh config
. get_config.sh $1/ema_config

ARMADILLO_PATH=$SCRIPT_DIR/$EXTERNALS_DIRECTORY/armadillo

cd $1/target

if [ -z "$BUILD_CONFIG" ]; then 
	BUILD_CONFIG=Release; 
fi


if [ -z "$GENERATOR" ]; then 
    cmake -DCMAKE_BUILD_TYPE=$BUILD_CONFIG -S cpp -B local_build
else 
    cmake -DCMAKE_BUILD_TYPE=$BUILD_CONFIG -G"$GENERATOR" -S cpp -B local_build
fi

cmake --build local_build

if [ "$DLL_DIR" != "." ]; then 
	cmake --install local_build
fi

popd
