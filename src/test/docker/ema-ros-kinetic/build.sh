curDir=$(readlink -f `dirname $0`)
docker build -t registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2middleware/ema-ros-kinetic $curDir
