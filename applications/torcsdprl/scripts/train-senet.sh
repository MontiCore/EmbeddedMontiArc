# generate network
cd affordance-networks/Senet
./senet-generate.sh

# train network
cd ../../
python target/senet/src/senet/cpp/CNNTrainer_senet.py

# copy log
./scripts/copy-log.sh SE-Net