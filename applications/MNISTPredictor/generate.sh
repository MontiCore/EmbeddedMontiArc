echo "Generating files.."
# (c) https://github.com/MontiCore/monticore  
java -jar embedded-montiarc-emadl-generator-0.2.6-SNAPSHOT-jar-with-dependencies.jar --models-dir src/main/emadl --root-model mnist.MnistClassifier --output-dir target --backend CAFFE2
