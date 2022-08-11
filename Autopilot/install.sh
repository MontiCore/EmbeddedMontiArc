#!/bin/bash

. ./config.sh

rm -rf target

echo "Generate agent..."
java -jar bin/*-jar-with-dependencies.jar config/agent.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${AGENT_BUILD}

mkdir "${BINARY}"

echo "Build agent..."
cmake -B${AGENT_BUILD} -Htarget/agent/src
make -C${AGENT_BUILD}
cp "${AGENT_BUILD}/de_rwth_montisim_agent_master/coordinator/Coordinator_de_rwth_montisim_agent_master" "${BINARY}/agent"
