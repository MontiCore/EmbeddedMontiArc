<!-- (c) https://github.com/MontiCore/monticore -->

# ShowAttendTellPredictor
This project aims at using the trained Show, attend and tell architecture for inference. It will predict a caption for a given input image. Read further for information on how to run the project.

## Prerequisites
1. Ubuntu Linux 16.04 LTS
2. Deep Learning Framework **MXNET** for Python and C++. (For installation instructions see https://mxnet.apache.org/get_started/ubuntu_setup ).
3. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
4. OpenCV


## How to Run
First, generate the python and C++ code of the net from the given EMADL models. Run:
```
bash build.sh
```

Then, run the predictor from the root directory of this project, specifying the path of the image you want to generate a caption for, and the dictionary to transform the predicted indices to words:
```
build/src/cpp/ShowAttendTell <image_path> <dictionary_path>
```

A concrete call could look like this:

```
build/src/cpp/ShowAttendTell resources/training_data/plane.jpg resources/training_data/dict.txt
```

Both PNG and JPEG images should work fine, and will be resized to the correct size of 224 x 224 pixels automatically.


