#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
TARGET_OUTPUT="target"
BINARY_OUTPUT="${TARGET_OUTPUT}/bin"

PROJECT_ROOT="$(pwd)"

AGENT_TRAINING_ROOT="${TARGET_OUTPUT}/src/${ROOT_COMPONENT_NAME}/cpp"
INSTANCE_NAME_NEURAL_NET="torcs_agent_mastercomponent_dpnet"
ENVIRONMENT_NAME="ros-torcs"

GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.1.1-20210507.114151-1-jar-with-dependencies.jar"

EXEC_ENVIRONMENT_TRAINING="python ./src/main/ros-gym-torcs/launcher.py"

CONFIG_AGENT="src/config/agent_config_cv.json"

AGENT_ROOT_COMPONENT="torcs_agent_mastercomponent"

AGENT_NEURAL_NET_NAME="Mastercomponent"

AGENT_BUILD="${TARGET_OUTPUT}/agent/build"

