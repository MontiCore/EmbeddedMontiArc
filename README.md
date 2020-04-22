<!-- (c) https://github.com/MontiCore/monticore -->

# BallTracking

## Prerequisites
1. Ubuntu Linux 16.04 LTS or 18.04 LTS (experimental)
2. Armadillo (at least armadillo version 6.600 must be used) [Official instructions at Armadillo Website](http://arma.sourceforge.net/download.html).
3. OpenCV.


## How to Run
Generate and build the code for the EMAM model by executing:

```
bash build.sh
```

Finally, run the BallTracking-Video as follows:
```
./build/src/cpp/BallTracking
```

You can try different videos to track a yellow object or object with different color by setting the corresponding values to CVInRange-variables in _BallTracking.cpp_.