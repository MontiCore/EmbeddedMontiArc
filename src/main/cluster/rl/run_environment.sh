#!/bin/bash

. config.sh

echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 4

echo "Start up environment..."
xterm -title "Gym-Environment" -e "python bin/ros-gym/launcher.py --environment \"TopoEnv\"; bash" &
sleep 2
