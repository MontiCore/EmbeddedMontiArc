#!/usr/bin/env bash
#
# ******************************************************************************
#  MontiCAR Modeling Family, www.se-rwth.de
#  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
#  All rights reserved.
#
#  This project is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 3.0 of the License, or (at your option) any later version.
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this project. If not, see <http://www.gnu.org/licenses/>.
# *******************************************************************************
#

current_dir=$(pwd)
parent_dir="$(dirname $current_dir)"
cp -r $parent_dir/lib/ $current_dir/lib


# create network for both autopilot and test containers
docker network create simulation-network

# start autopilot
docker run \
    --rm \
    --network simulation-network \
    -d \
    --name=autopilot \
    registry.git.rwth-aachen.de/monticore/embeddedmontiarc/simulators/rmimodelserver:latest \
    java -cp rmi-model-server.jar -Djava.rmi.server.codebase=file:rmi-model-server.jar \
    -Djava.rmi.server.useLocalHostname rwth.rmi.model.server.RMIServer 10101 autopilots --no-zookeeper no_time os_linux

# start integration test
docker run \
    --rm \
    --network simulation-network \
    -v ${parentdir}:/app \
    simulation-integration-test:latest \
    mvn install -s settings.xml -DskipTests &&\
    mvn -f runner/pom.xml -s settings.xml -Dtests=RunnerTest test"

# clean up
rm -rf $current_dir/lib
# remove the network that we just created
docker network rm simulation-network
# remove all containers
docker rm $(docker ps -aq)
# remove all images
docker rmi $(docker images -q)
