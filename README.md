# MNISTPredictor


## Prerequisites
1. Ubuntu Linux 16.04 LTS or 18.04 LTS (experimental)
2. Deep Learning Framework **Caffe2**. [Follow the instructions from this link](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2Caffe2#ubuntu).
3. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
4. OpenCV


## How to Run
First, build the MNISTPredictor by:

```
bash build.sh 
```

Then, navigate to the target directory in order to run the predictor as follows:
```
cd target/
./DigitClassifier ../src/resources/test_img/3.jpg
```

Try different images from the provided `/src/resources/test_img/` directory.


## Troubleshooting Help

ERROR: HelperA.h:79:28: error: ‘sqrtmat’ was not declared in this scope.

FIX:
Copy compiled armadillo lib and include files to usr/lib and usr/include respectively. Replace YOUR_ARMADILLO_REPOSITORY and VERSION (e.g. 8.500.1) with your corresponding information:
```
cd YOUR_ARMADILLO_REPOSITORY/armadillo-VERSION
sudo cp libarmadillo* /usr/lib
cd YOUR_ARMADILLO_REPOSITORY/armadillo-VERSION/include
sudo cp -r * /usr/include

