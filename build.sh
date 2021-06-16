# (c) https://github.com/MontiCore/monticore  

if [ ! -f /usr/lib/libmxnet.so ]; then
    echo "libmxnet.so not found at default location"
    echo "It should be there if the python mxnet package is installed"
    echo "Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly"
    exit 1
fi

rm -rf target
java -jar embedded-montiarc-emadl-generator-0.4.0-jar-with-dependencies.jar -m src/main/emadl/models -r mnistvae.Connector -o target -b GLUON

rm -rf build
mkdir build && cd build

echo "Building MNISTVAE.."
cmake -D MXNET_PATH=/usr/lib/libmxnet.so ..
make
