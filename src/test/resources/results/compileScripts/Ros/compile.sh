#!/usr/bin/env bash
set -e

# add *_HOME to PATH temporarily
if [ -n "$cmake_HOME" ]
then
   export PATH="$cmake_HOME:$PATH"
fi
if [ -n "$make_HOME" ]
then
   export PATH="$make_HOME:$PATH"
fi

# source additional environment variables
source "$ROS_HOME"/setup.bash

# check if needed programs are in PATH
if [[ `command -v cmake` ]]
then
    echo "Found cmake"
else
    echo "Can not find cmake in PATH! Aborting."
    exit 1
fi
if [[ `command -v make` ]]
then
    echo "Found make"
else
    echo "Can not find make in PATH! Aborting."
    exit 1
fi


# cmake
curDir=`dirname "$0"`
cmake -B"$curDir"/build/ -H"$curDir/src" "$@"

# make
make -C "$curDir"