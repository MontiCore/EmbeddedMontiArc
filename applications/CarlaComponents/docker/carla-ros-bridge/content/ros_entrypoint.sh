#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
set -e

# setup ros environment
source "/opt/carla-ros-bridge/install/setup.bash"
exec "$@"
