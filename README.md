<!-- (c) https://github.com/MontiCore/monticore -->
# Calculator MXNET


## Prerequisites
1. Ubuntu Linux 16.04 LTS or 18.04 LTS (experimental)
2. Deep Learning Framework **MXNet**
3. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
4. OpenCV

## Training the Network
train the network with calling .build.sh

## Generating Numbers
Afer training handwritten digits can be generated with: ./build/src/cpp/MNISTGenerator x y
where x is an integer number used as the random seed and y being an integer from 0-9.
