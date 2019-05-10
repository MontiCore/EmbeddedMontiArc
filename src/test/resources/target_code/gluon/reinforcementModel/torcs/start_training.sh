#!/bin/bash
cd reward/pylib
mkdir build
cd build
cmake ..
make
mv torcs_agent_dqn_reward_executor.py ../../../reinforcement_learning
mv _torcs_agent_dqn_reward_executor.so ../../../reinforcement_learning
cd ../../../
python CNNTrainer_torcs_agent_torcsAgent_dqn.py