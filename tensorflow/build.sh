rm -rf target
java -jar ../embedded-montiarc-emadl-generator-0.4.1-jar-with-dependencies.jar -m src/emadl/models/ -r cNNCalculator.Connector -o target -b TENSORFLOW -c n

rm -rf build
mkdir build && cd build
echo "Building DigitClassifier.."
cmake ..
make
