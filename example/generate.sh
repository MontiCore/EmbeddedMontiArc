echo "-- Generating files and training Models:"

java -jar embedded-montiarc-emadl-generator-0.2.5-SNAPSHOT-jar-with-dependencies.jar -m src/emadl/models -r cNNCalculator.Connector -o target -b MXNET

