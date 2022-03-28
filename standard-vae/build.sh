# (c) https://github.com/MontiCore/monticore

MXNET_PATH=$(python3 -c "import mxnet; print(mxnet.__file__)")
python3 -c "import mxnet; print(mxnet.__version__)"
MXNET_FOLDER=$(dirname $MXNET_PATH)

if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then
    echo "libmxnet.so not found at default location" $MXNET_FOLDER
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
fi

mvn dependency:resolve emadl:train -s settings.xml

#java -jar ../embedded-montiarc-emadl-generator-0.5.5-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl -r vae.Connector -o target -b GLUON -p /usr/bin/python3

rm -rf build
mkdir build && cd build
echo "MXNET_PATH: ${MXNET_PATH}"
echo "Building mnistvae.."
cmake -D MXNET_PATH=$MXNET_FOLDER/libmxnet.so ..
make