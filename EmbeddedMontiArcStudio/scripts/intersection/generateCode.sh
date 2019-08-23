#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
pushd `pwd` > /dev/null
cd $ROS_SIM_HOME

java -cp generator/embedded-montiarc-math-middleware-generator-0.0.1-SNAPSHOT-jar-with-dependencies.jar \
de.monticore.lang.monticar.generator.middleware.DistributedTargetGeneratorCli \
--models-dir=${HOME}/model/intersection \
--root-model=ba.system \
--output-dir=target/src/ \
--generators=cpp,roscpp

popd > /dev/null
