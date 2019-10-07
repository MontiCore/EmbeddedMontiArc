#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

. config.sh

if [ ! -d pendulum_logs ]
then
    mkdir pendulum_logs
fi

echo "Start ROSCORE..."
xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

echo "Start up environment..." 
xterm -title "Gym-Environment" -e "python bin/ros-gym/launcher.py --environment \"Pendulum-v0\" --quiet --continuous; bash" &
sleep 2

echo "Start up postprocessor..."
xterm -title "Postprocessor" -e "${BINARY}/postprocessor; bash" &
sleep 2

# Start the training
cd "target/agent/src/pendulum_agent_master/cpp"
python CNNTrainer_pendulum_agent_master_actor.py

cp -r ./model/* ${PROJECT_ROOT}/pendulum_logs/
cp -r ./model ${PROJECT_ROOT}/target/bin
cd "${PROJECT_ROOT}"
