<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2x/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2x/badges/master/coverage.svg)

# CNNArch2X

The MontiAnna generator toolchain is capable of generating C++ and Python code based on several well-known frameworks given a NN model architecture, a training configuration, and a dataset. The code generation is delegated by the [EMADL Generator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP). The EMADL Generator forward the composed model to the chosen framework-specific generators, called **[CNNArch2Gluon](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2Gluon)**, **[CNNArch2MXNet](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2MXNet)**, **[CNNArch2Caffe2](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2Caffe2)** and **[CNNArch2TensorFlow](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2tensorflow)**. All the framework-specific generators are an extension of the generator **CNNArch2X**, which facilitates the extension with further framework generators besides Gluon, MXNet, Caffe2, and TensorFlow. Each of the generators will generate a network creator, network trainer, and network API artifact in Python or C++. 

![generation-toolchain](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2x/badges/master/emadl_generator_toolchain.png)