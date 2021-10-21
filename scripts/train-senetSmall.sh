# generate network
cd affordance-networks/SenetSmall
./senetSmall-generate.sh

# train network
cd ../../
python target/senetSmall/src/senetSmall/cpp/CNNTrainer_senetSmall.py

# copy log
./scripts/copy-log.sh SE-Net-Small