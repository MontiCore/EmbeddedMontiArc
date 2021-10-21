# generate network
cd affordance-networks/ResnetSmall
./generate-resnetSmall.sh

# train network
cd ../../
python target/resnetSmall/src/resnetSmall/cpp/CNNTrainer_resnetSmall.py

# copy log
./scripts/copy-log.sh ResNetSmall