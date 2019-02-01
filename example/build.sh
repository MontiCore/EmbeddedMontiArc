rm -rf build
mkdir build && cd build

echo "Building DigitClassifier.."
cmake ..
cp -r /home/christopher/Documents/Master/Semester2/Praktikum/mxnet/include/mxnet/ ../target/
make