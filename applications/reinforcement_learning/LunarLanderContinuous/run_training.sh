#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
PROJECT_ROOT="$(pwd)"
TRAINING_ROOT="target/src/lander_master/cpp"

if [ ! -d target ]
then
    echo "No target folder found. Please run the generator and compile the application"
    exit 1
fi

# Start the training
cd "${TRAINING_ROOT}"
python CNNTrainer_lander_master_lander.py

cp -r ./model ${PROJECT_ROOT}/target/bin
cd "${PROJECT_ROOT}"