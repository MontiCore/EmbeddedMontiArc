rm -rf target
java -jar embedded-montiarc-emadl-generator-0.4.3-SNAPSHOT-jar-with-dependencies.jar -m src/main/emam/ -r ba.ballTracking -o target -flag-generate-cmake

rm -rf build
mkdir build && cd build

echo "Building BallTracking.."
cmake ..
make
