#!/bin/bash

./run_env.sh &
pid_env=$!
sleep 2


exitfn () {
    trap SIGINT
    kill $pid_env
    exit
}

trap "exitfn" INT

./run_training.sh

exitfn()
