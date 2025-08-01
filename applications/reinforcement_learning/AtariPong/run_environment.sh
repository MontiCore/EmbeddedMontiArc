#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

echo "Start up environment..." 
xterm -title "Gym-Environment" -e "python bin/ros-gym/launcher.py --environment \"Pong-v0\" --quiet; bash" &
sleep 2

echo "Start up preprocessor..."
xterm -title "Preprocessor" -e "${BINARY}/preprocessor; bash" &
sleep 2
