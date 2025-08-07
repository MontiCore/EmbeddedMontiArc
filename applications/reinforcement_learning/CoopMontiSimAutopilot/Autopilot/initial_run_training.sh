#!/bin/bash

(sleep 30s; pkill -9 -P $(pgrep run_training.sh)) &
pgrep ros | xargs kill -9
. /opt/ros/noetic/setup.sh
./run_training.sh "$@"
sleep 40
kill -9 $(pgrep -f 'python3 CNNTrainer_de_rwth_montisim_agent_master_qnet.py')
pgrep ros | xargs kill -9
. /opt/ros/noetic/setup.sh
./run_training.sh "$@"
