echo "Executing framework.."
# (c) https://github.com/MontiCore/monticore  
java -jar embedded-montiarc-emadl-generator-0.5.7-SNAPSHOT-jar-with-dependencies.jar -m src/main/emadl -r mnist.MnistClassifier -o target -b PYTORCH -f n -c n
echo "Finished executing framework"
/bin/bash