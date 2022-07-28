#!/bin/bash
source /ros_entrypoint.sh
roscore &
pid_ros=$!
sleep 3s

./trainEnvironment.sh &
pid_env=$!
sleep 2s

./trainAgent.sh

kill $pid_env
kill $pid_ros
