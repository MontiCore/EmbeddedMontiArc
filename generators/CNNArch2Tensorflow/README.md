<!-- (c) https://github.com/MontiCore/monticore -->
# CNNArch2Tensorflow

The Tensorlflow backend.

# New functionality
This backend has some funhctionality that other backend don't have:
A new layer (UpConvolution, see CNNArchLang and the flownet application)
A new training parameter (LossWeights, see CNNTrainLand and flownet application) This parameter can be used to give muliple losses diffrent weights, when having multiple outputs)

# Remarks
When using multiple outputs note that later inthe tamplates these outputs don't show up in the order they were defined in the emadl file, but in the order they were used). This is a general behaviour not
specific to this backend.

# Installation
For instructions on the installation of the prerequisites of this backend see in the EMADL2CPP backend.