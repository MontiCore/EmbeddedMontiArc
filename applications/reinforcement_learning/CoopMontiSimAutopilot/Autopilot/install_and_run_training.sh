#!/bin/bash

. /opt/ros/noetic/setup.sh

. ./config.sh
./install.sh

if [[ "$*" == *"-sp"* ]]
then
	/bin/echo "STARTING INITIAL RUN"
	./initial_run_training.sh "$@"
else
	./run_training.sh "$@"
fi

sleep 1
