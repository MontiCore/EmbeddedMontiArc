#!/bin/bash
BASEDIR=$(dirname $0)

echo "Remove target/alexnet"
rm -rf ../../target/alexnet-small
echo "Generate Code"
java -jar ../../bin/embedded-montiarc-math-middleware-generator-0.1.1-20210507.114151-1-jar-with-dependencies.jar alexnet-small-config.json
