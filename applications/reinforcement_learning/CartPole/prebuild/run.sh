#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

echo "Start agent..."
xterm -title "agent" -e "./agent -t 30; bash" &
sleep 1

echo "Start up environment..."
python ../bin/ros-gym/launcher.py --environment CartPole-v0 --eval
