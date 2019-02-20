#!/bin/bash
set -e
chmod +x ./target/generated-sources-ros2/distributed/compile.sh
chmod +x ./target/generated-sources-ros2/distributedStruct/compile.sh
chmod +x ./target/generated-sources-ros2/system/compile.sh
chmod +x ./target/generated-sources-ros2/addComp/compile.sh

./target/generated-sources-ros2/distributed/compile.sh
#TODO: struct compilation
#./target/generated-sources-ros2/distributedStruct/compile.sh
#./target/generated-sources-ros2/system/compile.sh
./target/generated-sources-ros2/addComp/compile.sh
