# 3d-gan

This is the repository for the 3D-GAN Project modeled in EMADL.
3D-GAN generates diverse objects from different categories. The files created have the Wavefront OBJ file format. You can use any exisiting visualization program to open these files. 
This repo contains pretrained parameters and the model files that can be used for EMADL.
**The 3D Convolution layers used on this model only run on GPUs.**

You can either use the provided visualize.py script in Pre-Trained/resources/Scripts to generate an object or the compiled program called with any number.

To change the object category for the pre-trained model, change the path in Pre-Trained/pom.xml to any of the available models under Pre-Trained/resources/model/{res_32 or res_64}/.
So to generate a bathtub, change 
` <path>resources/model/res_32/sofa</path>` to
`<path>resources/model/res_64/bathtub</path>`.

![Example of a chair](images/xampleChair.png)

![Example of a monitor](images/xampleMonitor.png)

![Example of a generated sofa](images/xampleSofa3.png)

![Another sofa](images/xampleSofa.png)
