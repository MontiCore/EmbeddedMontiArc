#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
echo "Starting simulation..."
cd ../shared
source variables.sh
source /opt/ros/kinetic/setup.sh
cd ../intersection

#Hack to enable graphical output  
export DISPLAY=":0"

set -e

./clean.sh
./generateCode.sh
./compile.sh
./startSimulation.sh
