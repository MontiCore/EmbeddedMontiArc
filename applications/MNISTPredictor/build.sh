rm -rf build
# (c) https://github.com/MontiCore/monticore
mkdir build && cd build

echo "Building DigitClassifier.."
cmake ..
make

cp src/main/cpp/DigitClassifier ../target/DigitClassifier

echo "Copying resources to target directory.."
cd ..
cp -r src/main/resources/* ./target
echo "done"

