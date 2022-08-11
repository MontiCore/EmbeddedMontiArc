#!/bin/bash

rosdep update
. /opt/ros/noetic/setup.sh

. ./config.sh
./install.sh

if [[ "$*" == *"-sp"* ]]
then
	./initial_run_training.sh > ./output.txt
else
	./run_training.sh > ./output.txt
fi

sleep 1
