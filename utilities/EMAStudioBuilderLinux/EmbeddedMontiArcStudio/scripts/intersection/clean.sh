#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
if [ ! -z $ROS_SIM_HOME ]; then
	echo "cleaning $ROS_SIM_HOME/target" 
	rm -rf $ROS_SIM_HOME/target
	mkdir $ROS_SIM_HOME/target
else
	echo "ERROR: ROS_SIM_HOME is undefined!"
fi
