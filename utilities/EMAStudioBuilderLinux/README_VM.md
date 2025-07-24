<!-- (c) https://github.com/MontiCore/monticore -->
# EMAStudio
To start EMAStudio, double click EMAStudio.sh and choose *Run in Terminal* or start *ide.sh* in */home/user/EMAStudio/* and wait for chrome to open. 

## Classifier
To start the Classifier example, click on the checkmark below *EmbeddedMontiArcStudio Classifier*.

The *classifier* classifies images of the [cifar10](https://www.cs.toronto.edu/~kriz/cifar.html) dataset (32x32 color images with 10 classes).
Use the button *Execute Model* to apply the trained classifier to test images. Test images are selected by drag-and-drop.
The button *Rebuild Project* will remove the existing classifer and generate, compile and train a new classifer based on the EMADL model.
The data for training can be found in the directory "build/EMAStudio/EmbeddedMontiArcStudio/data".


## Intersection
To start the Intersection example, click on the checkmark below *EmbeddedMontiArcStudio Intersection*.

The simulation can be started by clicking on the *Execute Model* button. The progress can then be seen in the terminal. After the simulator is fully loaded, focus the window *rqt_reconfigure__Param - rqt*, click on *time_mgmt* in the list on the left, and uncheck the pause_time variable. The vehicles will now drive through each other once. After that the controller is active and will prevent further crashes.
