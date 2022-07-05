#!/bin/bash
rm -rf runLog.txt
java -jar embedded-montiarc-emadl-generator-0.5.6-SNAPSHOT-jar-with-dependencies.jar -m /home/martin/Development/Masterarbeit/mnistcalculator/mnist-calculator/src/main/ema/emadl/models -r cNNCalculator.Connector -o mnist-calculator/target -b gluon >> runLog.txt
