#!/bin/bash

. /opt/ros/kinetic/setup.sh

cd $HOME/Dokumente/toolchain/rl/
./install.sh
./run_all.sh > ../logs/training_log.txt
sleep 1