#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

echo "Start agent..."
xterm -title "agent" -e "./agent; bash" &
sleep 2

python ../../bin/ros-gym/launcher.py --environment BipedalWalker-v2 --continuous --eval
