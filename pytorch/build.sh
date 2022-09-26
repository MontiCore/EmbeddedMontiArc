echo "Copying resources to target directory.."
cp -r src/main/resources/* target
cp -r src/main/cpp/* target
echo "Done copying"

rm -rf build
# (c) https://github.com/MontiCore/monticore
mkdir build && cd build

echo "Building DigitClassifier.."
cmake -DCMAKE_PREFIX_PATH=/usr/lib/libtorch/  ..
cmake --build . --config Release

/bin/bash


