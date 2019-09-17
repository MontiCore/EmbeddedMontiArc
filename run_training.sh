#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

. config.sh

if [ ! -d "${TARGET_OUTPUT}" ] \
    || [ ! -d "${BINARY_OUTPUT}" ] \
    || [ ! -f "${BINARY_OUTPUT}/agent" ];
then
    echo "Cannot find binaries. Please run install.sh first..."
    exit 1
fi

echo "Start training..."
cd "${TARGET_OUTPUT}/agent/src/${AGENT_ROOT_COMPONENT}/cpp"
sh start_training.sh
cp -r ./model "${PROJECT_ROOT}/${BINARY_OUTPUT}"
cd ${PROJECT_ROOT}
