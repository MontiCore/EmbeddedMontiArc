rm -rf target
java -jar embedded-montiarc-math-generator-0.1.16-SNAPSHOT-jar-with-dependencies.jar -m src/main/emam/ -r ba.ballTracking -o target -flag-generate-cmake

rm -rf build
mkdir build && cd build

echo "Building BallTracking.."
cmake ..
make
