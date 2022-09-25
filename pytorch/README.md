<!-- (c) https://github.com/MontiCore/monticore -->
# MNISTPredictor


## Prerequisites
1. Ubuntu Linux 20.04 LTS (experimental)
2. Deep Learning Framework **PyTorch**. [Follow the instructions from this link](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2pytorch).
3. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
4. OpenCV


## How to Run
First, generate the code for the EMADL model by executing:
```
bash generate.sh
```

Then, build the MNISTPredictor by:

```
bash build.sh
```

Finally, navigate to the target directory in order to run the predictor as follows:
```
cd target/
./DigitClassifier ./test_img/3.jpg
```

You can try different images from the provided `/test_img` directory or even create your own images for testing. Note that the application program DigitClassifier will automatically rescale the given input image to the size 28x28.


## Troubleshooting Help

ERROR: HelperA.h:79:28: error: ‘sqrtmat’ was not declared in this scope.

FIX:
Copy compiled armadillo lib and include files to usr/lib and usr/include respectively. Replace YOUR_ARMADILLO_REPOSITORY and VERSION (e.g. 8.500.1) with your corresponding information:
```
cd YOUR_ARMADILLO_REPOSITORY/armadillo-VERSION
sudo cp libarmadillo* /usr/lib
cd YOUR_ARMADILLO_REPOSITORY/armadillo-VERSION/include
sudo cp -r * /usr/include

