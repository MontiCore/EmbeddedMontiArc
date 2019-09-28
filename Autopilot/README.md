<!-- (c) https://github.com/MontiCore/monticore -->
## How to use the wrapper:

1.  Build the EMAM Component. See [this page](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/carlacomponents/blob/master/README.md) for more information.

2.  Install (and start) the carla-ros-bridge, as described [here](https://github.com/carla-simulator/ros-bridge).

3.  Install (and start) the carla-waypoint-publisher, as described [here](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher).

4.  Start the python script by running:

        python converter.py 

    on your shell while in the containing directory.

5.  Start the executable generated in step one by running:

        cd ./target/install/bin
        ./Coordinator_wrapper_wrapper
        
    while in the main directory of this project.