#!/bin/bash

. config.sh

echo "Start ROSCORE..."
roscore &
sleep 4

echo "Start up environment..."
python bin/ros-gym/launcher.py --environment \"TopoEnv\" &
sleep 2
