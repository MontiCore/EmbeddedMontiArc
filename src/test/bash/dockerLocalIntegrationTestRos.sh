#!/usr/bin/env bash
baseDir=$(readlink -f `dirname $0`/../../..)
docker run -t -v $baseDir:/project registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2middleware/ema-ros-kinetic /bin/bash -c "cd project; src/test/bash/integrationTestRos.sh"
chown -R `who am i | awk '{print $1}'` "$baseDir/target/generated-sources-cmake/"