echo "Generating files.."
# (c) https://github.com/MontiCore/monticore  
java -jar embedded-montiarc-emadl-generator-0.5.6-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl -r mnist.MnistClassifier -o target -b PYTORCH -f n -c n
echo "Done generating files"
/bin/bash