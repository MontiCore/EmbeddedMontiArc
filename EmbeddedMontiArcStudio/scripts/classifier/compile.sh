pushd `pwd` > /dev/null
source "../shared/variables.sh"
echo "Compiling image classifier.."
cd ${HOME}
cp scripts/classifier/ImageClassifier.cpp ${CLASSIFIER_CPP}/ImageClassifier.cpp
cd ${CLASSIFIER_CPP}
g++ -std=c++11 ImageClassifier.cpp -o ImageClassifier -I. -I${MXNET_HOME}/include -I${ARMADILLO_HOME}/include -L${ARMADILLO_HOME} -L${MXNET_HOME}/lib -lmxnet -llapack -lblas -larmadillo  `pkg-config --cflags --libs opencv`
mkdir -p ${CLASSIFIER_TARGET}
cp ${CLASSIFIER_CPP}/ImageClassifier ${CLASSIFIER_TARGET}/ImageClassifier
popd  > /dev/null
