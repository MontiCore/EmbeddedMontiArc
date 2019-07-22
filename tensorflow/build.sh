rm -rf target
java -jar ../embedded-montiarc-emadl-generator-0.2.13-jar-with-dependencies.jar -m src/emadl/models/ -r cNNCalculator.Connector -o target -b TENSORFLOW

rm -rf build
mkdir build && cd build

echo "Building DigitClassifier.."
cmake ..
make
