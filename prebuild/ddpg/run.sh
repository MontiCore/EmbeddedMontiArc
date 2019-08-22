#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2


echo "Start postprocessor..."
xterm -title "Postprocessor" -e "./postprocessor; bash" &
sleep 2

echo "Start agent..."
xterm -title "Agent" -e "./agent; bash" &
sleep 2

echo "Start up environment..."
python ../../bin/ros-gym/launcher.py --environment "Pendulum-v0" --continuous --eval
