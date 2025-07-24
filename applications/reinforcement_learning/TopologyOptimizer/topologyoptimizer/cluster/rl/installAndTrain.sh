#!/bin/bash

. /opt/ros/noetic/setup.sh

SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
cd $SCRIPT_DIR
./install.sh
./run_all.sh
