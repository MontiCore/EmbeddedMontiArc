#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

. config.sh

# Start the training
cd "target/agent/src/atari_agent_master/cpp"
python CNNTrainer_atari_agent_master_dqn.py

cp -r ./model ${PROJECT_ROOT}/${BINARY}
cd "${PROJECT_ROOT}"
