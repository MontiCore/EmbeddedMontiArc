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

# check if needed programs are in PATH
if [[ `command -v cmake` ]]
then
	echo "Found cmake"
else
	echo "Can not find cmake in PATH! Aborting."
	echo "Try setting the environment variable cmake_HOME to the base of your installation or adding it to your PATH!"
	exit 1
fi
if [[ `command -v make` ]]
then
	echo "Found make"
else
	echo "Can not find make in PATH! Aborting."
	echo "Try setting the environment variable make_HOME to the base of your installation or adding it to your PATH!"
	exit 1
fi

# source additional environment variables
source "$ROS2_HOME"/setup.bash

# Post source check if needed programs are in PATH
if [[ `command -v ros2` ]]
then
	echo "Found ros2"
else
	echo "Can not find ros2 in PATH! Aborting."
	echo "Try setting the environment variable ROS2_HOME to the base of your installation or adding it to your PATH!"
	exit 1
fi

# cmake
curDir=`dirname "$0"`
cmake -B"$curDir"/build/ -H"$curDir/src" "$@"

# make
make -j4 -C "$curDir/build"