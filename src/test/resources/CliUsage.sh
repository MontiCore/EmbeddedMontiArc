#!/usr/bin/env bash
java -cp target/embedded-montiarc-math-middleware-generator-0.0.1-SNAPSHOT-jar-with-dependencies.jar \
de.monticore.lang.monticar.generator.middleware.DistributedTargetGeneratorCli \
--models-dir=src/test/resources/ \
--root-model=tests.a.addComp \
--output-dir=target/cli-test/src \
--generators=cpp,roscpp