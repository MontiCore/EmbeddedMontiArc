winpty docker run \
# (c) https://github.com/MontiCore/monticore  
-it \
--rm \
--name carlacomponents \
--mount type=bind,source=//c/Users/Public/Documents/SharedFolder/carlacomponents,target=/usr/src/carlacomponents \
registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/carlacomponents/emam-carla-ros-bridge \
#bash
