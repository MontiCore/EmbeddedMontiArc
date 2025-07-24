#!/bin/bash

. config.sh

mkdir target/bin/model

cp -r training_results/training_stage_2/* target/bin/model
sleep 2
echo "Start the gazebo simulator"
roslaunch turtlebot3_gazebo turtlebot3_world.launch &
sleep 10

echo "Start up preprocessor..."
cd "${PROJECT_ROOT}/${BINARY}"
./preprocessor -executeOnDemand &
sleep 5

echo "Start up postprocessor..."
./postprocessor -executeOnDemand &
sleep 5

echo "Start up agent..."
./agent -executeOnDemand &
sleep 5

cd "${PROJECT_ROOT}/${BINARY_GAZEBO}"
echo "Start up gazebo environment..."
python roslauncher.py --environment gazeboEnv -p exp1
sleep 6
