#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh
AGENT_BUILD="target/agent/atari_agent_master/build"
PREPROCESSOR_BUILD="target/preprocessor/atari_preprocessor_master/build"
mkdir "${BINARY}"
cp "${PREPROCESSOR_BUILD}/atari_preprocessor_master/coordinator/Coordinator_atari_preprocessor_master" "${BINARY}/preprocessor"


cp "${AGENT_BUILD}/atari_agent_master/coordinator/Coordinator_atari_agent_master" "${BINARY}/agent"

echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

echo "Start up environment..." 
xterm -title "Gym-Environment" -e "python bin/ros-gym/launcher.py --environment \"Pong-v0\" --quiet; bash" &
sleep 2



echo "Start up preprocessor..."
xterm -title "Preprocessor" -e "${BINARY}/preprocessor; bash" &
sleep 2
