#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.0.33-SNAPSHOT-jar-with-dependencies.jar"
PREPROCESSOR_BUILD="target/preprocessor/build"
AGENT_BUILD="target/agent/build"

rm -rf target

echo "Generate preprocessor..."
java -jar ${GENERATOR_PATH} config/preprocessor.json

echo "Generate agent..."
java -jar ${GENERATOR_PATH} config/agent.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${PREPROCESSOR_BUILD}
rm -rf ${AGENT_BUILD}

mkdir "${BINARY}"

echo "Build preprocess..."
cmake -B${PREPROCESSOR_BUILD} -Htarget/preprocessor/src
make -C${PREPROCESSOR_BUILD}
cp "${PREPROCESSOR_BUILD}/atari_preprocessor_master/coordinator/Coordinator_atari_preprocessor_master" "${BINARY}/preprocessor"

echo "Build agent..."
cmake -B${AGENT_BUILD} -Htarget/agent/src
make -C${AGENT_BUILD}
cp "${AGENT_BUILD}/atari_agent_master/coordinator/Coordinator_atari_agent_master" "${BINARY}/agent"
