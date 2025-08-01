#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
. config.sh

# Clear target directory
echo "Clear target folder"
rm -rf ${TARGET_OUTPUT}

echo "Generate files..."

# Genrate Agent
echo "Generate agent"
java -jar "${GENERATOR_PATH}" "${CONFIG_AGENT}"


if [ ! -d "${TARGET_OUTPUT}" ] || [ ! -d "${TARGET_OUTPUT}/agent" ]
then
    echo "No ${TARGET_OUTPUT} found"
    exit 1
fi

echo "Start building..."
echo "Clean up build files"
rm -rf "${BINARY_OUTPUT}"
rm -rf "${AGENT_BUILD}"
mkdir target/bin

# Build agent
echo "Building agent..."
cmake -B"${AGENT_BUILD}" -H"${TARGET_OUTPUT}/agent/src"
make -C "${AGENT_BUILD}"
cp "${AGENT_BUILD}/${AGENT_ROOT_COMPONENT}/coordinator/Coordinator_${AGENT_ROOT_COMPONENT}" "${BINARY_OUTPUT}/agent"
