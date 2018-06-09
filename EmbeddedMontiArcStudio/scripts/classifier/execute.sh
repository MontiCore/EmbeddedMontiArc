pushd `pwd` > /dev/null
source "../shared/variables.sh"
cd ${CLASSIFIER_TARGET}
./ImageClassifier ./img.png
popd  > /dev/null
