#!/bin/bash

SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
cd $SCRIPT_DIR

. $SCRIPT_DIR/config.sh

rm -rf target

echo "Generate agent..."
java -jar bin/*-jar-with-dependencies-int.jar config/agent.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${AGENT_BUILD}

mkdir "${BINARY}"

echo "Build agent..."
cmake -B"${AGENT_BUILD}" -Htarget/agent/src
make -C "${AGENT_BUILD}"
cp "${AGENT_BUILD}/topology_agent_master/coordinator/Coordinator_topology_agent_master" "${BINARY}/agent"
