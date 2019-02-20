baseDir=$(readlink -f `dirname $0`/../../..)
docker run -v $baseDir:/project registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2middleware/ema-ros-crystal /bin/bash -c "cd project; src/test/bash/integrationTestRos2.sh"
