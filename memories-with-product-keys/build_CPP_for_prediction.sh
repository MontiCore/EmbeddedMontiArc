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

rm -rf build
mkdir build && cd build

echo "Building MemoriesWithProductKeys.."
cmake -D MXNET_PATH=$MXNET_FOLDER/libmxnet.so ..
#cmake -D MXNET_PATH=$MXNET_FOLDER/libmxnet.so -DCMAKE_BUILD_TYPE=Debug -G CodeBlocks - Ninja ..
make
