rm -rf target
java -jar embedded-montiarc-math-generator-0.1.16-SNAPSHOT-jar-with-dependencies.jar -m src/main/emam/ -r ba.ballTracking -o target -flag-generate-cmake
cd target
sed -i 's/cube/Cube<unsigned char>/g' ba_ballTracking.h
cd ..
rm -rf build
mkdir build && cd build

echo "Building BallTracking.."
cmake ..
make
