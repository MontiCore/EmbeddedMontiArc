#!/bin/bash

if nvcc --version; then
    echo command returned true
else
    module load cuda/100
fi
module load python/3.6.8


PYTHONPATH=$(which python3)

MXNET_PATH=$(python3 -c "import mxnet; print(mxnet.__file__)")
MXNET_FOLDER=$(dirname $MXNET_PATH)
echo $MXNET_FOLDER

if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then
    echo "libmxnet.so not found at default location" $MXNET_FOLDER
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
    exit 1
fi

EMADL_GEN_PATH=/home/treiber/.m2/repository/de/monticore/lang/monticar/embedded-montiarc-emadl-generator/0.3.8-SNAPSHOT/embedded-montiarc-emadl-generator-0.3.8-SNAPSHOT-jar-with-dependencies.jar
if test -f "$EMADL_GEN_PATH"; then
    echo "EMADL Generator Path: " $EMADL_GEN_PATH
else
    EMADL_GEN_PATH=../embedded-montiarc-emadl-generator-0.3.8-SNAPSHOT-jar-with-dependencies.jar
    echo "EMADL Generator Path: " $EMADL_GEN_PATH
fi

# rm -rf target
# java -jar $EMADL_GEN_PATH -m src/emadl/models/ -r encoderDecoder.Connector -o target -b GLUON -p $PYTHONPATH