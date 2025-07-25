pushd `pwd` > /dev/null
# (c) https://github.com/MontiCore/monticore  
source "../shared/variables.sh"
if [ -e "${CLASSIFIER_TARGET}/ImageClassifier" ] && [ -d "${CLASSIFIER_TARGET}/model" ]
then
  echo "ImageClassifier and trained classifier already exists. Skipping generation."
else
  echo "Generating files.."
  cd ${HOME}
  java -jar emadl2cpp.jar -m model/classifier -r cifar10.Main -o ${CLASSIFIER_CPP}

  mkdir -p ${CLASSIFIER_TRAIN}
  for entry in "${CLASSIFIER_CPP}"/*
  do
    if [[ ${entry} = *".py" ]]; then
      mv ${entry} ${CLASSIFIER_TRAIN}/${entry##*/}
    fi
  done
fi
popd  > /dev/null
