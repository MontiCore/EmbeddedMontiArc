# (c) https://github.com/MontiCore/monticore

source ../setup_itc.sh

rm -rf target
java -jar $EMADL_GEN_PATH -m src/emadl/models/ -r cNNSegment.Connector -o target -b GLUON -p $PYTHONPATH
