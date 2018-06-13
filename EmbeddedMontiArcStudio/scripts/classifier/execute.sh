pushd `pwd` > /dev/null
source "../shared/variables.sh"
cd ${CLASSIFIER_TARGET}
chmod +x ImageClassifier
./ImageClassifier ./img.png
popd  > /dev/null
