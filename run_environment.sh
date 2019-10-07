#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 4

echo "Start up environment..." 
eval ${EXEC_ENVIRONMENT_TRAINING};
