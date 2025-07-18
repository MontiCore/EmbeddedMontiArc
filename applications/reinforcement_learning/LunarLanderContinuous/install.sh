#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
if [ ! -d "target" ];
then
    echo "No target folder found..."
    echo "Generate project..."
    rm -rf target
    java -jar bin/embedded-montiarc-math-middleware-generator-0.0.33-SNAPSHOT-jar-with-dependencies.jar config.json
fi
echo "Target folder found!"
echo "Build project..."
rm -rf target/build
rm -rf target/bin
cmake -Btarget/build -Htarget/src
make -Ctarget/build

mkdir target/bin
cp target/build/lander_master/coordinator/Coordinator_lander_master target/bin/agent
