#!/bin/bash

PROJECT_ROOT="$(pwd)"
BINARY="target/bin"
GENERATOR_PATH="bin/embedded-montiarc-math-middleware-generator-0.1.4-SNAPSHOT-jar-with-dependencies.jar"
AGENT_BUILD="target/agent/build"
SIMULATOR_PATH="$HOME/dev/basic-simulator/install"
SCENARIO_PATH="$PROJECT_ROOT/scenario_file.json"
SAVE_FOLDER="$HOME/dev/coopmontisimautopilot/saved_autopilots"
SELF_PLAY_AGENT_EXECUTION_INTERVAL_TRAINING=5 # in milliseconds. Adjust for your system. The lower, the better. Only applies to training. You should only see a handful of "Cant keep up" messages as a maximum in the output
SELF_PLAY_AGENT_EXECUTION_INTERVAL_RUNNING=100 # in milliseconds. Set equal to the simulation step in the scenario file. Only applies to the execution
