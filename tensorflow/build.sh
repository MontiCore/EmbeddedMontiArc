rm -rf target
java -jar ../embedded-montiarc-emadl-generator-0.3.6-SNAPSHOT-jar-with-dependencies.jar -m src/emadl/models/ -r flowNetS.Network -o target -b TENSORFLOW -f n

rm -rf build
mkdir build && cd build

echo "Building FlowNetS.."
cmake ..
make
