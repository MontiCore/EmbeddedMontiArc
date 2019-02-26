#!/bin/bash
#
#
#  ******************************************************************************
#  MontiCAR Modeling Family, www.se-rwth.de
#  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
#  All rights reserved.
#
#  This project is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 3.0 of the License, or (at your option) any later version.
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this project. If not, see <http://www.gnu.org/licenses/>.
# *******************************************************************************
#

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