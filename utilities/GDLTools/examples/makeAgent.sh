#!/bin/bash

cd ./*agent
dos2unix *.sh
source /ros_entrypoint.sh
./install.sh
