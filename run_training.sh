#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
if [ ! -d logs ]
then
    mkdir logs
fi

PROJECT_ROOT="$(pwd)"
TRAINING_ROOT="target/src/cheetah_master/cpp"

if [ ! -d target ]
then
    echo "No target folder found. Please run the generator and compile the application"
    exit 1
fi

# Start the training
cd "${TRAINING_ROOT}"
python3 CNNTrainer_cheetah_master_cheetah.py

cp -r ./model/WalkerActor/* ${PROJECT_ROOT}/logs
cp -r ./model ${PROJECT_ROOT}/target/bin
cd "${PROJECT_ROOT}"
