curDir=$(readlink -f `dirname $0`)
# (c) https://github.com/MontiCore/monticore  
docker build -t registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/carlacomponents/emam-carla-ros-bridge:latest -f Dockerfile ./.. "$@"
