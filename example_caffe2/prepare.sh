echo "Creating HDF5 dataset from image files.."
cd resources/
wget https://git.rwth-aachen.de/thomas.timmermanns/EMADL-Demo/raw/master/src/resources/data.tar.gz
tar xf data.tar.gz
rm data.tar.gz
python imgDir-to-h5.py --in_port data --out_port softmax --data_path data --target_path training_data
rm -r data