#!/bin/bash


echo "Start the simulator"
#roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
roslaunch turtlebot3_gazebo turtlebot3_world.launch

echo "Start up environment..."
python bin/roslauncher.py --environment \"gazeboEnv\"

echo "Start up preprocessor..."
cd "${PROJECT_ROOT}/${BINARY}"
./preprocessor -executeOnDemand &