rm -rf target
java -jar ../embedded-montiarc-emadl-generator-0.3.3-SNAPSHOT-jar-with-dependencies.jar -m src/emadl/models/ -r endtoend.Nvidia -o target -b TENSORFLOW

rm -rf build
mkdir build && cd build

