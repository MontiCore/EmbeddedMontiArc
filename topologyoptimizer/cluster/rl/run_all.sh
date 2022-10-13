#!/bin/bash

echo "Run environment..."
nohup ./run_environment.sh &
sleep 4

echo "Run training..."
nohup ./run_training.sh &
sleep 2

wait
