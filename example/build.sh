rm -r target/; 
java -jar embedded-montiarc-emadl-generator-0.2.5-SNAPSHOT-jar-with-dependencies.jar -m src/emadl/models/ -r cNNCalculator.Connector -o target -b MXNET

rm -rf build
mkdir build && cd build

echo "Building DigitClassifier.."
cmake ..
#cp -r /home/christopher/Documents/Master/Semester2/Praktikum/mxnet/include/mxnet/ ../target/
make