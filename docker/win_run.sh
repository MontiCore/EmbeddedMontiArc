winpty docker run \
-it \
--rm \
--name emam2carla2 \
--mount type=bind,source=//c/Documents/Code/carlacomponents,target=/usr/src/emam2carla \
registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/carlacomponents/emam-carla-ros-bridge \
bash
