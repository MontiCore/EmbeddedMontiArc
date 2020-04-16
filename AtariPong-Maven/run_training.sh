#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

. config.sh

# Start the training
cd "target/agent/atari_agent_master/src/atari_agent_master/emadlcpp"
python CNNTrainer_atari_agent_master_dqn.py

#cp -r ../../../../../../src/main/ema/model ${PROJECT_ROOT}/${BINARY}
#cd "${PROJECT_ROOT}"
