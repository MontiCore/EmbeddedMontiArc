<!-- (c) https://github.com/MontiCore/monticore -->
# Documentation
This documentation includes basic information about the implemented concepts and the required installation steps for the MontiSim project.

# Required Software
- Apache Maven
- Java Development Kit, Version 8+
- Browser with WebGL support (recommended: Chrome)
- _(optional)_ Git
- _(optional)_ Java IDE (recommended: IntelliJ IDEA)

# Installation Steps
To install the distributed MontiSim simulator, one needs to install the [server](https://github.com/MontiSim/server) and [visualization](https://github.com/MontiSim/visualization) applications from their respective repositories. Please follow the instructions given in each of those repositories to setup, build and execute the distributed simulator.

For users __downloading__ the above mentioned repositories as _.zip_ files, rather than clonning them via `git`, we remind that those projects have to be extracted from the archive files and any further configuration regarding these projects have to be made from within the __root directory__ of the respective project.

# Demo Server
- http://137.226.168.10/visualization/
- __Note:__ available only in RWTH Network

# Additional usage tips

* For Chrome there is a useful tip:

    If your CPU can not create the simulation results fast enough for the visualization, the visualization will begin to stutter. To avoid that, you can click the `Start` button and afterwards you can minimize your Chrome browser window for some time. This allows the simulation to compute the required results without having to display them immediately. After some waiting you can open the browser window again and the visualization should run smoothly.
