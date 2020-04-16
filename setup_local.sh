MXNET_PATH=$(python -c "import mxnet; print(mxnet.__file__)")
MXNET_FOLDER=$(dirname $MXNET_PATH)
echo $MXNET_FOLDER

EMADL_GEN_PATH=/home/treiber/.m2/repository/de/monticore/lang/monticar/embedded-montiarc-emadl-generator/0.3.8-SNAPSHOT/embedded-montiarc-emadl-generator-0.4.0-jar-with-dependencies.jar
if test -f "$EMADL_GEN_PATH"; then
    echo "EMADL Generator Path: " $EMADL_GEN_PATH
else
    EMADL_GEN_PATH=../embedded-montiarc-emadl-generator-0.4.0-jar-with-dependencies.jar
    echo "EMADL Generator Path: " $EMADL_GEN_PATH
fi