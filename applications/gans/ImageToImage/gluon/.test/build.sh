MXNET_PATH=$(python -c "import mxnet; print(mxnet.__file__)")
# (c) https://github.com/MontiCore/monticore  
MXNET_FOLDER=$(dirname $MXNET_PATH)
echo $MXNET_FOLDER

if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then
    echo "libmxnet.so not found at default location" $MXNET_FOLDER
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
fi

rm -rf target
java -jar resources/embedded-montiarc-emadl-generator-0.3.8-jar-with-dependencies.jar -m src/emadl/models/ -r imageToImage.Connector -o target -b GLUON

