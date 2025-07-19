# generate network
cd affordance-networks/SenetSmallFC
./senetSmallFC-generate.sh

# train network
cd ../../
python target/senetSmallFC/src/senetSmallFC/cpp/CNNTrainer_senetSmallFC.py

# copy log
./scripts/copy-log.sh SE-Net-SmallFC