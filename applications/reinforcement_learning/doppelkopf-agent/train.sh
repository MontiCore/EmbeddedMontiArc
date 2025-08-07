#!/bin/bash
source /ros_entrypoint.sh
roscore > /dev/null &
pid_ros=$!
sleep 3s

./trainEnvironment.sh > /dev/null &
pid_env=$!
sleep 2s

exitfn () {
    trap SIGINT
    kill $pid_env
    kill $pid_ros
    exit
}

trap "exitfn" INT

./trainAgent.sh

exitfn()
