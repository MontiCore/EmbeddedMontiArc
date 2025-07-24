pushd `pwd` > /dev/null
# (c) https://github.com/MontiCore/monticore  
source "../shared/variables.sh"
source ${USER_HOME}/mxnet/bin/activate
if [ -d "${CLASSIFIER_TARGET}/model" ]
then
  echo "Trained model already exists. Skipping training."
else
  echo "Creating HDF5 dataset from image files.."
  mkdir -p ${CLASSIFIER_TRAIN}/data/cifar10_main_net
  python ${HOME}/scripts/classifier/imgDir-to-h5.py --in_port data --out_port softmax --data_path ${DATA_DIR} --target_path ${CLASSIFIER_TRAIN}/data/cifar10_main_net

  echo "Start network training.."
  cd ${CLASSIFIER_TRAIN}
  python CNNTrainer_cifar10_Main.py
  mkdir -p ${CLASSIFIER_TARGET}/model/cifar10_main_net
  cp ${CLASSIFIER_TRAIN}/model/cifar10_main_net/net_newest-symbol.json ${CLASSIFIER_TARGET}/model/cifar10_main_net/net_newest-symbol.json
  cp ${CLASSIFIER_TRAIN}/model/cifar10_main_net/net_newest-0000.params ${CLASSIFIER_TARGET}/model/cifar10_main_net/net_newest-0000.params
fi
popd  > /dev/null
