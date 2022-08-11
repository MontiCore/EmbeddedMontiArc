#!/bin/bash

(sleep 30s; pkill -9 -P $(pgrep run_training.sh)) &
./run_training.sh -sp
sleep 40
kill -9 $(pgrep -f 'python CNNTrainer_de_rwth_montisim_agent_master_qnet.py')
./run_training.sh -sp
