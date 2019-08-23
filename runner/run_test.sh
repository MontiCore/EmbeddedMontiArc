#!/usr/bin/env bash

current_dir=$(pwd)
parent_dir="$(dirname $current_dir)"
cp -r $parent_dir/lib/fmu_for_linux/*.fmu $current_dir/lib/

# kill the containers if they are still running
docker kill integration-test
docker rm integration-test
docker kill autopilot
docker rm autopilot

# Build the image
docker build -t simulation-integration-test -f $current_dir/Dockerfile ../

# create network for both autopilot and test containers
docker network create simulation-network

# start autopilot
docker run \
    --rm \
    --network=simulation-network \
    -d \
    --name=autopilot \
    registry.git.rwth-aachen.de/monticore/embeddedmontiarc/simulators/rmimodelserver:latest \
    java -cp rmi-model-server.jar -Djava.rmi.server.codebase=file:rmi-model-server.jar \
    -Djava.rmi.server.useLocalHostname rwth.rmi.model.server.RMIServer 10101 autopilots --no-zookeeper no_time os_linux

# start integration test
docker run \
    --rm \
    --name=integration-test \
    --network=simulation-network \
    simulation-integration-test:latest \
    sh -c "mvn -f runner/pom.xml -s settings.xml -Dtests=RunnerTest test"

# clean up
rm  $current_dir/lib/*.fmu
# remove the network that we just created
docker network rm simulation-network
# remove test image
docker rmi simulation-integration-test
