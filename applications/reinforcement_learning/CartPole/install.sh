#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.0.33-SNAPSHOT-jar-with-dependencies.jar"
AGENT_BUILD="target/agent/build"

rm -rf target

echo "Generate agent..."
java -jar ${GENERATOR_PATH} config/agent.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${AGENT_BUILD}

mkdir "${BINARY}"

echo "Build agent..."
cmake -B${AGENT_BUILD} -Htarget/agent/src
make -C${AGENT_BUILD}
cp "${AGENT_BUILD}/cartpole_agent_master/coordinator/Coordinator_cartpole_agent_master" "${BINARY}/agent"
