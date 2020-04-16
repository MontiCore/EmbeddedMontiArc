<!-- (c) https://github.com/MontiCore/monticore -->
# MNISTGenerator

## Prerequisites
1. Ubuntu Linux 16.04 LTS or 18.04 LTS (experimental)
3. Deep Learning Framework **MXNet**.
4. Armadillo (at least armadillo version 9. must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
4. OpenCV 

## Training the Network
Train with calling ./build.sh inside the gluon folder.

## Generating Numbers
After training handwritten digits can be generated with: ./build/src/cpp/MNISTGenerator x 
where x is an integer number used as the random seed.
