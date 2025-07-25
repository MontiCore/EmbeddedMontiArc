# generate network
cd affordance-networks/AlexnetSmall
./generate-alexnet-small.sh

# train network
cd ../../
python target/alexnet-small/src/alexnetSmall/cpp/CNNTrainer_alexnetSmall.py

# copy log
./scripts/copy-log.sh AlexNet-Small