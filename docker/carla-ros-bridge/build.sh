#!/bin/sh
# (c) https://github.com/MontiCore/monticore  

docker build -t carla-ros-bridge -f Dockerfile ./.. "$@"
