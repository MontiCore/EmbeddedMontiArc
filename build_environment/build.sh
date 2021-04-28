#!/bin/bash

if [ -z "$CMAKE" ]; then 
    echo "This script must be used inside the DSA image."
    exit 0
fi

if [ -z "$1" ] || [ "$1" == "--help" ]; then 
    echo "Usage:"
    echo "    ./build.sh <folder>"
    echo "Description:"
    echo "    Builds the CMake project inside the specified folder."
    exit 0
fi


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

pushd $SCRIPT_DIR/$1

[ ! -d "build" ] && mkdir build
cd build

$CMAKE ..
make

popd