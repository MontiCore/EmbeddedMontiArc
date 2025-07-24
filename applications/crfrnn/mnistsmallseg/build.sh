# (c) https://github.com/MontiCore/monticore
MXNET_PATH=$(python -c "import mxnet; print(mxnet.__file__)")
MXNET_FOLDER=$(dirname $MXNET_PATH)
echo $MXNET_FOLDER

if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then
    echo "libmxnet.so not found at default location" $MXNET_FOLDER
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
    exit 1
fi

# EMADL_GEN_PATH=/home/treiber/.m2/repository/de/monticore/lang/monticar/embedded-montiarc-emadl-generator/0.3.8-SNAPSHOT/embedded-montiarc-emadl-generator-0.4.0-jar-with-dependencies.jar
# if test -f "$EMADL_GEN_PATH"; then
#     echo "EMADL Generator Path: " $EMADL_GEN_PATH
# else
#     EMADL_GEN_PATH=../embedded-montiarc-emadl-generator-0.4.0-jar-with-dependencies.jar
#     echo "EMADL Generator Path: " $EMADL_GEN_PATH
# fi

EMADL_GEN_PATH=../embedded-montiarc-emadl-generator-0.4.0-jar-with-dependencies.jar

rm -rf target
java -jar $EMADL_GEN_PATH -m src/emadl/models/ -r cNNSegment.Connector -o target -b GLUON

rm -rf build
mkdir build && cd build

echo "Building DigitSegment.."
cmake -D MXNET_PATH=$MXNET_FOLDER/libmxnet.so ..
make
