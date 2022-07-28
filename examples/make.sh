#!/bin/bash

./makeAgent.sh &
pid=$!

./makeEnvironment.sh
wait $pid
