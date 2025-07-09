# Letter Predictor
The Letter Predictor model is a derivation of the [MNIST-Calculator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/mnistcalculator) and [MNIST-Predictor models](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/mnistpredictor). 
This model can predict handwritten letters.
The dataset included is the [EMNIST letter dataset](https://www.nist.gov/itl/products-and-services/emnist-dataset) converted in to the HDF5 format.
To extract the images and build the `.h5` files the `H5Generator.py` script was used.
Additionally, there is a C++ application to execute the model.

