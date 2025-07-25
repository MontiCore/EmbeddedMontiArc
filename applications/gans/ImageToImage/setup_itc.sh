#!/usr/local_rwth/bin/zsh


### load necessary modules
if nvcc --version; then
    which nvcc
else
    module load cuda/100
    module load cudnn/7.4
fi
module load python/3.6.8
module load cmake

PYTHONPATH=$(which python3)

echo "Python path:" $PYTHONPATH

MXNET_PATH=$($PYTHONPATH -c "import mxnet; print(mxnet.__file__)")
MXNET_FOLDER=$(dirname $MXNET_PATH)
echo $MXNET_FOLDER

if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then
    echo "libmxnet.so not found at default location" $MXNET_FOLDER
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
    exit 1
fi

EMADL_GEN_PATH=/home/jt529748/.m2/repository/de/monticore/lang/monticar/embedded-montiarc-emadl-generator/0.3.8-SNAPSHOT/embedded-montiarc-emadl-generator-0.3.8-SNAPSHOT-jar-with-dependencies.jar
if test -f "$EMADL_GEN_PATH"; then
    echo "EMADL Generator Path: " $EMADL_GEN_PATH
else
    EMADL_GEN_PATH=../embedded-montiarc-emadl-generator-0.3.8-SNAPSHOT-jar-with-dependencies.jar
    echo "EMADL Generator Path: " $EMADL_GEN_PATH
fi
