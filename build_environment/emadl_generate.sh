#!/bin/bash

if [ -z "$1" ] || [ "$1" == "--help" ]; then
    echo "Usage:"
    echo "    ./emadl_generate.sh 'folder'"
    echo "Description:"
    echo "    Generates the CPP files for the EMADL project specified in the 'folder'/ema_config file."
    echo "Configuration:"
    echo "    The 'EMADL_TO_CPP_PROJECT' and 'EMADL_TO_CPP_VERSION' properties must be set inside the 'config' file."
    exit 0
fi


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $SCRIPT_DIR

. get_config.sh config
. get_config.sh $1/ema_config

EMADL_TO_CPP_JAR=$EMADL_TO_CPP_PROJECT/target/embedded-montiarc-emadl-generator-$EMADL_TO_CPP_VERSION-jar-with-dependencies.jar

cd $1

OUTPUT_DIR=target/cpp

java -jar $EMADL_TO_CPP_JAR \
  --models-dir=$MODEL_DIR \
  --root-model=$MODEL_NAME \
  --output-dir=$OUTPUT_DIR \
  --flag-use-armadillo-backend \
  --flag-generate-cmake \
  --armadillo-import \
  --tcp-adapter \
  --output-name=$OUTPUT_NAME \
  --library-interface 

popd
