pushd `pwd` > /dev/null
# (c) https://github.com/MontiCore/monticore  
source "../shared/variables.sh"
cd ${CLASSIFIER_TARGET}
chmod +x ImageClassifier
./ImageClassifier ./img.png
popd  > /dev/null
