#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
pushd `pwd` > /dev/null
if [ -z $ROS_SIM_HOME ]; then
	echo "ERROR: ROS_SIM_HOME is not set. Aborting!"
	exit 1
fi

cd $ROS_SIM_HOME

mkdir target/build
cd target/build
#fix for missing armadillo.h; To be replaced by FindArmadillo.cmake
export CXXFLAGS=-isystem\ "$ARMADILLO_HOME/include"
cmake ../src
make -j4

popd > /dev/null
