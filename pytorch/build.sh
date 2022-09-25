echo "Copying resources to target directory.."
cp -r src/main/resources/* target
echo "Done copying"

rm -rf build
# (c) https://github.com/MontiCore/monticore
mkdir build && cd build

echo "Building DigitClassifier.."
cmake --build . --config Release

/bin/bash


