# (c) https://github.com/MontiCore/monticore  
:'
if [ ! -f /usr/lib/libmxnet.so ]; then
    echo "libmxnet.so not found at default location"
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
    exit 1
fi

rm -rf target
cp -R resources target

chmod -R 777 target


rm -rf build
mkdir build && cd build

echo "Building MNISTVAE.."
cmake -D MXNET_PATH=/usr/lib/libmxnet.so ..
make '
MXNET_PATH=$(python3 -c "import mxnet; print(mxnet.__file__)")
python3 -c "import mxnet; print(mxnet.__version__)"
# (c) https://github.com/MontiCore/monticore
MXNET_FOLDER=$(dirname $MXNET_PATH)

if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then
    echo "libmxnet.so not found at default location" $MXNET_FOLDER
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
fi

rm -rf target
java -jar /home/asmeta/.m2/repository/de/monticore/lang/monticar/embedded-montiarc-emadl-generator/0.6.0-SNAPSHOT/embedded-montiarc-emadl-generator-0.6.0-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl/models -r mnistvae.Connector -o target -b GLUON -p /usr/bin/python3

rm -rf build
mkdir build && cd build

echo "Building MNISTVAE.."
cmake -D MXNET_PATH=$MXNET_FOLDER/libmxnet.so ..
make

