#!/bin/bash

if [ -z "$1" ] || [ "$1" == "--help" ]; then
    echo "Usage:"
    echo "    ./build_local.sh 'folder' [Config]"
    echo "Description:"
    echo "    Builds the project for the local architecture."
    echo "    Config: optional, specifies Release or Debug, defaults to Release"
    exit 0
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

pushd $SCRIPT_DIR

. get_config.sh config

SHARED_CPP_DIR=$SCRIPT_DIR/$EXTERNALS_DIRECTORY/shared_cpp
CMAKE_ARGS="-DSHARED_CPP_DIR=${SHARED_CPP_DIR}"

BUILD_CONFIG="${2}"
if [ -z "$BUILD_CONFIG" ]; then 
	BUILD_CONFIG=Release; 
fi
CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=${BUILD_CONFIG}"
BUILD_DIR="local_${BUILD_CONFIG}"

if [ -n "$GENERATOR" ]; then
    CMAKE_ARGS="${CMAKE_ARGS} -G\"${GENERATOR}\""
fi

echo "CMAKE_ARGS=$CMAKE_ARGS"


cd $1

cmake ${CMAKE_ARGS} -S . -B ${BUILD_DIR}
cmake --build ${BUILD_DIR}

popd
