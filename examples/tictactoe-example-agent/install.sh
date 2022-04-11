#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

GENERATOR_PATH={$EMAM_GENERATOR}
AGENT_BUILD="generator-target/agent/build"

rm -rf generator-target

echo "Generate ${NAME_OF_AGENT} agent..."
java -jar ${GENERATOR_PATH} config/agent.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${AGENT_BUILD}

mkdir "${BINARY}"

echo "Build agent..."
cmake -B${AGENT_BUILD} -Hgenerator-target/agent/src
make -C${AGENT_BUILD}

cp "${AGENT_BUILD}/${PACKAGE_NAME}_agent_master/coordinator/Coordinator_${PACKAGE_NAME}_agent_master" "${BINARY}/agent"

sed -i "4i matplotlib.use('Agg')" generator-target/agent/src/${PACKAGE_NAME}_agent_master/cpp/reinforcement_learning/util.py
sed -i "4i import matplotlib" generator-target/agent/src/${PACKAGE_NAME}_agent_master/cpp/reinforcement_learning/util.py