#!/bin/bash

xterm -title "Training" -e "./run_training.sh; bash" &
sleep 2
xterm -title "Rosgym" -e "./run_environment.sh; bash" &
sleep 2
