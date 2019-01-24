rm -rf build
mkdir build && cd build

echo "Building DigitClassifier.."
cmake ..
make

cp src/resources/DigitClassifier ../target/DigitClassifier

