#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
TARGET_OUTPUT="target"
BINARY_OUTPUT="${TARGET_OUTPUT}/bin"

PROJECT_ROOT="$(pwd)"

AGENT_TRAINING_ROOT="${TARGET_OUTPUT}/src/${ROOT_COMPONENT_NAME}/cpp"
INSTANCE_NAME_NEURAL_NET="torcs_agent_torcsAgent_actor"
ENVIRONMENT_NAME="ros-torcs"

GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.0.33-SNAPSHOT-jar-with-dependencies.jar"

EXEC_ENVIRONMENT_TRAINING="python ./bin/ros-gym-torcs/launcher.py"

CONFIG_AGENT="config/agent_config.json"

AGENT_ROOT_COMPONENT="torcs_agent_torcsAgent"

AGENT_NEURAL_NET_NAME="TorcsActor"

AGENT_BUILD="${TARGET_OUTPUT}/agent/build"
