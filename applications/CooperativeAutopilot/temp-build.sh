#!/bin/bash

pushd target/tmp

cmake -DARMADILLO_PATH=external/armadillo -S cpp/de.rwth.connectedcars.TestAutoPilot -B de.rwth.connectedcars.TestAutoPilot
cmake --build de.rwth.connectedcars.TestAutoPilot --config Release

popd