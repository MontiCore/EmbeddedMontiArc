#!/bin/bash
BASEDIR=$(dirname $0)

echo "Remove target/resnet"
rm -rf ../../target/resnet
echo "Generate Code"
java -jar ../../bin/embedded-montiarc-math-middleware-generator-0.1.1-20210507.114151-1-jar-with-dependencies.jar resnet-config.json