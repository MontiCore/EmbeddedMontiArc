<!-- (c) https://github.com/MontiCore/monticore -->
# MNISTGenerator MXNET


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

Finally, run the Generator as follows:
```
./build/src/cpp/DigitCalculator [arbitrary integer]
```

The integer is used as a seed for the random generator, thus different numbers result in different
created images.


## Troubleshooting Help

ERROR: HelperA.h:79:28: error: ‘sqrtmat’ was not declared in this scope.

FIX:
Copy compiled armadillo lib and include files to usr/lib and usr/include respectively. Replace YOUR_ARMADILLO_REPOSITORY and VERSION (e.g. 8.500.1) with your corresponding information:
```
cd YOUR_ARMADILLO_REPOSITORY/armadillo-VERSION
sudo cp libarmadillo* /usr/lib
cd YOUR_ARMADILLO_REPOSITORY/armadillo-VERSION/include
sudo cp -r * /usr/include
