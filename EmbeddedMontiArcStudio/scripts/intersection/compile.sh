#!/bin/bash
pushd `pwd` > /dev/null
if [ -z $ROS_SIM_HOME ]; then
	echo "ERROR: ROS_SIM_HOME is not set. Aborting!"
	exit 1
fi

cd $ROS_SIM_HOME

mkdir target/build
cd target/build
cmake ../src
make -j4

popd > /dev/null
