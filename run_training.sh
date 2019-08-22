#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

if [ ! -d logs ]
then
    mkdir logs
fi

# Start the training
cd "target/agent/src/lander_agent_master/cpp"
python CNNTrainer_lander_agent_master_dqn.py

cp -r ./model/* ${PROJECT_ROOT}/logs/
cp -r ./model ${PROJECT_ROOT}/target/bin
cd "${PROJECT_ROOT}"
