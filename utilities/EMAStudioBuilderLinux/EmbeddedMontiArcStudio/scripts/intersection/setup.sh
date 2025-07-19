#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
echo "Installing ROS"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init

sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

echo "Installing Simulation Framework dependencies"
sudo apt-get install -y ros-kinetic-geodesy ros-kinetic-tf2-geometry-msgs python-catkin-tools libpugixml-dev qt5-default cmake

echo "Compiling framework"
pushd `pwd` > /dev/null

curDir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_SIM_HOME=$curDir/../../RosSimulationFramework/

cd $ROS_SIM_HOME
cd catkin_ws

source /opt/ros/kinetic/setup.bash

catkin init
catkin clean -y
catkin build -DCMAKE_BUILD_TYPE=Release

popd
