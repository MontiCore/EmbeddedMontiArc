# End2EndDriving

This repository consists of a neural network for end to end driving and some useful tools. It implements [this paper](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) by Nvidia on End to End driving. In summary, the paper describes a method to predict steering angels for a given input image of a camera, which is mounted to the front of a car.

## Requirements

- OpenCV, h5py, numpy, PIL, matplotlib and scipy for Python: `pip install opencv h5py numpy PIL matplotlib scipy`
- OpenCV for C
- Tensorflow for C: [This repo might help you](https://github.com/FloopCZ/tensorflow_cc)

## Usage

- Go into tensorflow directory

### Training

- Generate `train.h5` and `test.h5` files and move them to resources/training_data. (See section Generation of Training Data). The pictures should have a dimensionality of 3x480x640 and the label *data*. The output is a steering angle with the label *target_label*.
- Train by executing `./build.sh`

### Prediction

|  | Predict | Visualise | Command |
|:---|:---|:---|:---|
| 1 | &#9745; | | `python3 v-tool/cli.py -i test.h5 -p` |
| 2 | &#9745; | &#9745; | `python3 v-tool/cli.py -i test.h5 -p -v` |
| 3 |  | &#9745; | `python3 v-tool/cli.py -i test.h5 -v` (execute 1 or 2 first)|

### Visualisation

Predictions are visualized through a Python GUI using Matplotlib. It shows the input frames (frontal camera), the according real steering angels and predicted steering angels as rotating wheels, and the predction error (in angels).
![Prediction Visualisations](ss.png)


## Generation of Training Data

Execute `python jpg2hdf5.py` in `tensorflow/resources/data_transform_scripts` to generate a small test data set. Create bigger data sets similiarly to the steps in this python file.



