#!/bin/bash

directory=$(pwd)
. $directory/config.sh

cd "target/agent/src/topology_agent_master/cpp"
python3 CNNTrainer_topology_agent_master_qnet.py

cp -r ./model ${PROJECT_ROOT}/${BINARY}
cd "${PROJECT_ROOT}"
