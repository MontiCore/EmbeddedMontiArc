#!/bin/bash
#source /ros_entrypoint.sh

#roscore &
#pid_ros=$!
#sleep 3s

./run_env.sh &
pid_env=$!
sleep 2s

./run_training.sh

kill $pid_env
kill $pid_ros