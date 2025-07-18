#!/bin/bash

. config.sh

echo "Start ROSCORE..."
roscore &
sleep 4

rostopic echo /topo/step > step.log &
rostopic echo /topo/state > state.log &
rostopic echo /topo/reward > reward.log &

echo "Start up environment..."
python bin/ros-gym/launcher.py --environment \"TopoEnv\" --verbose --render 1 &
sleep 2
