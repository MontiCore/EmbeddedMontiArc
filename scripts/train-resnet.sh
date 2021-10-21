# generate network
cd affordance-networks/Resnet
./generate-resnet.sh

# train network
cd ../../
python target/resnet/src/resnet/cpp/CNNTrainer_resnet.py

# copy log
./scripts/copy-log.sh ResNet