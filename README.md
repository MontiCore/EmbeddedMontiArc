# End2EndDriving

This repository consists of a neural network for end to end driving and some usefull tools. It implements the [paper](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) by Nvidia on End to End driving.

## Requirements

- OpenCV, h5py, numpy, PIL, matplotlib and scipy for Python: `pip install opencv h5py numpy PIL matplotlib scipy`
- OpenCV for C
- Tensorflow for C: [This repo might help you](https://github.com/FloopCZ/tensorflow_cc)

## Usage

- Go into tensorflow directory

### Training

- Generate `train.h5` and `test.5` files and move them to resources/training_data. (See section Generation of Training Data). The pictures should have a dimensionality of 3x480x640 and the label *data*. The output is a steering angle with the label target. 
- Train by executing `./build.sh`

### Prediction
- Predict with `./build/src/cpp/steeringAnglePredictor path_to_testh5`
- **or** Predict and visualise with `python3 v-tool/cli.py -i absolute_path_to_testh5 -p absolute_path_to_tensorflow_dir/build/src/cpp/steeringAnglePredictor`
- **or** Visualise with `python3 v-tool/cli.py -p absolute_path_to_tensorflow_dir/build/src/cpp/steeringAnglePredictor`. This requires execution of the previous command.


## Generation of Training Data

Execute `python jpg2hdf5.py` in `tensorflow/resources/data_transform_scripts` to generate a small test data set. Create bigger data sets similiarly to the steps in this python file.



