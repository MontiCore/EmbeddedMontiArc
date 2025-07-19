#!/bin/bash

nohup singularity exec /rwthfs/rz/SW/UTIL.common/singularity/dglqd_rlcontainercpu rl/installAndTrain.sh &
sleep 3
nohup python3 toolchain/files/dyna.py &
wait