#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.0.33-SNAPSHOT-jar-with-dependencies.jar"
AGENT_BUILD="target/agent/build"
POSTPROCESSOR_BUILD="target/postprocessor/build"

rm -rf target

echo "Generate agent..."
java -jar ${GENERATOR_PATH} config/agent.json

echo "Generate postprocessor..."
java -jar ${GENERATOR_PATH} config/postprocessor.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${AGENT_BUILD}
rm -rf ${POSTPROCESSOR_BUILD}

mkdir "${BINARY}"

echo "Build agent..."
cmake -B${AGENT_BUILD} -Htarget/agent/src
make -C${AGENT_BUILD}
cp "${AGENT_BUILD}/pendulum_agent_master/coordinator/Coordinator_pendulum_agent_master" "${BINARY}/agent"

echo "Build postprocessor..."
cmake -B${POSTPROCESSOR_BUILD} -Htarget/postprocessor/src
make -C${POSTPROCESSOR_BUILD}
cp "${POSTPROCESSOR_BUILD}/pendulum_postprocessor_master/coordinator/Coordinator_pendulum_postprocessor_master" "${BINARY}/postprocessor"
