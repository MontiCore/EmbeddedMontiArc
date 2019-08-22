#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
if [ "$EUID" -ne 0 ]
  then echo "Please run as root!"
  exit 1
fi

# ROS
# setup sources.list
echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# setup keys
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

# install bootstrap tools
apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools

rosdep init

apt-get install -y ros-kinetic-desktop-full

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc 

# coincar dependencies
apt-get install -y ros-kinetic-tf2-geometry-msgs ros-kinetic-geodesy python-catkin-tools python-wstool libgeographic-dev libpugixml-dev qt5-default

# java
apt-get install openjdk-8-jre

# Others
apt-get install -y unzip g++ wget
apt-get install -y libopenblas-dev libhdf5-serial-dev
