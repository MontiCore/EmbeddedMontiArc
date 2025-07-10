#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
pushd `pwd` > /dev/null
if [ -z $ROS_SIM_HOME ]; then
	echo "ERROR: ROS_SIM_HOME is not set. Aborting!"
	exit 1
fi

cd $ROS_SIM_HOME
export ROS_LOG_DIR=$ROS_SIM_HOME/logs
cd catkin_ws

echo "Starting framework"
source devel/setup.bash
systemFolder=../target/build
sleepTime=100
roslaunch simulation_initialization_ros_tool _whole_framework.launch & \
($systemFolder/ba_system_collisionDetection/coordinator/Coordinator_ba_system_collisionDetection -t $sleepTime) & \
($systemFolder/ba_system_intersectionController/coordinator/Coordinator_ba_system_intersectionController -t $sleepTime) & \
($systemFolder/ba_system_velocityController_1_/coordinator/Coordinator_ba_system_velocityController_1_ -t $sleepTime) & \
($systemFolder/ba_system_velocityController_2_/coordinator/Coordinator_ba_system_velocityController_2_ -t $sleepTime)

popd > /dev/null
