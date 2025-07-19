#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

GENERATOR_PATH="embedded-montiarc-math-middleware-generator-jar-with-dependencies.jar"
AGENT_BUILD="generator-target/agent/build"

rm -rf generator-target

echo "Generate tictactoe agent..."
java -jar ${GENERATOR_PATH} config/tictactoe-agent.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${AGENT_BUILD}

mkdir "${BINARY}"

echo "Build agent..."
cmake -B${AGENT_BUILD} -Hgenerator-target/agent/src
make -C${AGENT_BUILD}

cp "${AGENT_BUILD}/tictactoe_agent_master/coordinator/Coordinator_tictactoe_agent_master" "${BINARY}/agent"

sed -i "4i matplotlib.use('Agg')" generator-target/agent/src/tictactoe_agent_master/cpp/reinforcement_learning/util.py
sed -i "4i import matplotlib" generator-target/agent/src/tictactoe_agent_master/cpp/reinforcement_learning/util.py