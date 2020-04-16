<!-- (c) https://github.com/MontiCore/monticore -->
# Dummy Segmentation for MNIST with MXNET Gluon


## Prerequisites
1. Ubuntu Linux 16.04 LTS or 18.04 LTS (experimental)
2. Deep Learning Framework **MXNet**
3. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
4. OpenCV


## How to Run
Generate and build the code for the EMADL model by executing:

```
bash build.sh
```

Finally, run the segmentation as follows:
```
./build/src/cpp/DigitSegment ../resources/images/1.png
./build/src/cpp/DigitSegment ../resources/images_fashionmnist/1.png
```

You can try different images from the provided `/resources/images` directory or even create your own images for testing. Note that the application program DigitSegment will automatically rescale the given input image to the size 28x28.

Alternatively run provided python script to test network seperately:

```
python ./python/DigitSegment.py -i ../resources/images/1.png
python ./python/DigitSegment.py -i ../resources/images_fashionmnist/1.png

```