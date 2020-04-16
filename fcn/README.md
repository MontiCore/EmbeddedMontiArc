<!-- (c) https://github.com/MontiCore/monticore -->
# FCN for Semantic Segmentation


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
./build/src/cpp/DigitSegment ../resources/images_voc/1.png
```