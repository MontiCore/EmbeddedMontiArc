#!/bin/bash
echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

echo "Start agent..."
xterm -title "Agent" -e "./agent; bash" &
sleep 2

echo "Start up environment..."
python ../../bin/ros-gym/launcher.py --environment "LunarLanderContinuous-v2" --continuous --eval