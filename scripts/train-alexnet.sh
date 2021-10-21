# generate network
cd affordance-networks/Alexnet
./generate-alexnet.sh

# train network
cd ../../../target-training/src/dpnet/cpp
python CNNTrainer_dpnet.py
cd ../../
python target/alexnet/src/alexnet/cpp/CNNTrainer_alexnet.py

# copy log
./scripts/copy-log.sh AlexNet