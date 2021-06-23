#!/bin/bash

if [ -z "$1" ] || [ "$1" == "--help" ]; then 
    echo "Usage:"
    echo "    ./ema_build_local.sh <folder> [Config]"
    echo "Description:"
    echo "    Builds the EMA CMake project inside the specified folder."
    echo "    Local compile = for this machine."
    echo "    Config: optional, specifies Release or Debug, defaults to Release"
    exit 0
fi


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $SCRIPT_DIR

. get_config.sh config
. get_config.sh $1/ema_config

ARMADILLO_PATH=$SCRIPT_DIR/$EXTERNALS_DIRECTORY/armadillo
CMAKE_ARGS="-DARMADILLO_PATH=\"${ARMADILLO_PATH}\""

BUILD_CONFIG="${2}"
if [ -z "$BUILD_CONFIG" ]; then 
	BUILD_CONFIG=Release; 
fi
CMAKE_ARGS=" ${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=${BUILD_CONFIG}"
BUILD_DIR="local_${BUILD_CONFIG}"

if [ -n "$GENERATOR" ]; then
    CMAKE_ARGS=" ${CMAKE_ARGS} -G\"${GENERATOR}\""
fi

echo "CMAKE_ARGS=$CMAKE_ARGS"

cd $1/target

cmake $CMAKE_ARGS -S cpp -B ${BUILD_DIR}
cmake --build ${BUILD_DIR}

if [ "$DLL_DIR" != "." ]; then 
	cmake --install ${BUILD_DIR}
fi

popd
