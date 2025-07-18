cd affordance-networks/AlexnetLight
./generate-alexnet-light.sh

# train network
cd ../../
python target/alexnet-light/src/alexnetLight/cpp/CNNTrainer_alexnetLight.py

# copy log
./scripts/copy-log.sh AlexNet-Light