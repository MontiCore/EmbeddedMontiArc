# (c) https://github.com/MontiCore/monticore  

if [ ! -f /usr/lib/libmxnet.so ]; then
    echo "libmxnet.so not found at default location"
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
    exit 1
fi

rm -rf target
java -jar /../embedded-montiarc-emadl-generator-0.6.0-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl/models -r customloss.Connector -o target -b GLUON -p /usr/bin/python3 -cl customloss.L2

rm -rf build
mkdir build && cd build

echo "Building MNISTVAE.."
cmake -D MXNET_PATH=/usr/lib/libmxnet.so ..
make
