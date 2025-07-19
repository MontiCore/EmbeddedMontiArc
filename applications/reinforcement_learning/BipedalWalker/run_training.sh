#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
PROJECT_ROOT="$(pwd)"
TRAINING_ROOT="target/src/bipedalwalker_master/cpp"

if [ ! -d target ]
then
    echo "No target folder found. Please run the generator and compile the application"
    exit 1
fi

echo "Start ROSCORE..."
roscore &
sleep 2

echo "Start up environment..." 
python bin/ros-gym/launcher.py --environment "BipedalWalker-v3" --quiet --continuous &
sleep 2

# Start the training
cd "${TRAINING_ROOT}"
python CNNTrainer_bipedalwalker_master_walker.py

cp -r ./model ${PROJECT_ROOT}/target/bin
cd "${PROJECT_ROOT}"
