#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.1.4-SNAPSHOT-jar-with-dependencies.jar"
PREPROCESSOR_BUILD="target/preprocessor/build"
AGENT_BUILD="target/agent/build"
POSTPROCESSOR_BUILD="target/postprocessor/build"

rm -rf target

echo "Generate preprocessor..."
java -jar ${GENERATOR_PATH} config/preprocessor.json

echo "Generate agent..."
java -jar ${GENERATOR_PATH} config/agent.json

echo "Generate postprocessor..."
java -jar ${GENERATOR_PATH} config/postprocessor.json

echo "Building..."
rm -rf ${BINARY}
rm -rf ${PREPROCESSOR_BUILD}
rm -rf ${AGENT_BUILD}
rm -rf ${POSTPROCESSOR_BUILD}

mkdir "${BINARY}"

echo "Build preprocessor..."
cmake -B${PREPROCESSOR_BUILD} -Htarget/preprocessor/src
make -C${PREPROCESSOR_BUILD}
cp "${PREPROCESSOR_BUILD}/turtlebot_preprocessor_master/coordinator/Coordinator_turtlebot_preprocessor_master" "${BINARY}/preprocessor"


echo "Build agent..."
cmake -B${AGENT_BUILD} -Htarget/agent/src
make -C${AGENT_BUILD}
cp "${AGENT_BUILD}/turtlebot_agent_master/coordinator/Coordinator_turtlebot_agent_master" "${BINARY}/agent"


echo "Build postprocessor..."
cmake -B${POSTPROCESSOR_BUILD} -Htarget/postprocessor/src
make -C${POSTPROCESSOR_BUILD}
cp "${POSTPROCESSOR_BUILD}/turtlebot_postprocessor_master/coordinator/Coordinator_turtlebot_postprocessor_master" "${BINARY}/postprocessor"

sed -i "4i matplotlib.use('Agg')" target/agent/src/${NAME_OF_AGENT}_agent_master/cpp/reinforcement_learning/util.py
sed -i "4i import matplotlib" target/agent/src/${NAME_OF_AGENT}_agent_master/cpp/reinforcement_learning/util.py

sed -i "s:import reinforcement_learning.util as util:import util:g" target/agent/src/${NAME_OF_AGENT}_agent_master/cpp/reinforcement_learning/cnnarch_logger.py