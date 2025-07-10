#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

# exit with code on error
set -e

source variables.sh
export STREAM_TESTING="${PROJECT_ROOT}/target/generated-sources-cpp/streamtest"
export AUTOPILOT_TESTS="${STREAM_TESTING}/target/generated-sources-cpp/streamtest/autopilot"
export STREAM_TEST_EXEC_DIR="${STREAM_TESTING}/exec"

g++ -std=c++11 \
   -I"${JAVA_HOME}/include_linux" \
   -I"${JAVA_HOME}/include_linux/linux" \
   -I"${ARMADILLO_HOME}/include" \
   -L"${HOME}/lib/linux" \
   -o "${2}/TestsForCurrentModel" \
   "${1}/test/tests_main.cpp" \
   -include "${ARMADILLO_HOME}/include/armadillo" \
   -DARMA_DONT_USE_WRAPPER -lopenblas
