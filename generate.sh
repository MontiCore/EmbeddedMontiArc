echo "Generating files.."
java -jar embedded-montiarc-emadl-generator-0.2.6-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl -r mnist.MnistClassifier -o target -b CAFFE2
