cd affordance-networks/AlexnetSmall
./generate-alexnet-small.sh

cd ../../../target-training/src/dpnet/cpp
python CNNTrainer_dpnet.py
cd ../../
python target/alexnet-small/src/alexnetSmall/cpp/CNNTrainer_alexnetSmall.py

# copy log
./scripts/copy-log.sh AlexNetSmall