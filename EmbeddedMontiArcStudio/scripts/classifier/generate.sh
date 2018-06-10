pushd `pwd` > /dev/null
source "../shared/variables.sh"
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
popd  > /dev/null
