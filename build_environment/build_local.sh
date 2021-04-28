#!/bin/bash

if [ -z "$1" ] || [ "$1" == "--help" ]; then
    echo "Usage:"
    echo "    ./build_local.sh 'folder'"
    echo "Description:"
    echo "    Builds the project for the local architecture. "
    exit 0
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

pushd $SCRIPT_DIR

. get_config.sh config

SHARED_CPP_PATH=../../externals/shared_cpp

cd $1

cmake -S . -B local_build
cmake --build local_build

popd
