<!-- (c) https://github.com/MontiCore/monticore -->
## Steps to build a component:

1.  Move **mw-generator.jar** to the directory of the component you wish to build.
2.  Open shell, cd to the directory of the component and execute: 

        java -jar mw-generator.jar project.json
        
    (in this case it's called **valid.json** or **settings.json**)
3.  Switch to the target directory:

        cd target/
4.  Execute compile.sh:

         ./compile.sh
5.  If the generator can't find carla_msgs message type: execute the following command and retry step 4:

        export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/path/to/rosbridge/catkin_ws/devel/share/carla_msgs/cmake
       
    usually the path is something like `~/carla-ros-bridge/catkin_ws/devel/share/carla_msgs/cmake`.
6.  After successfull compiling the generated code switch to install/bin directory:

        cd install/bin/
7.  Execute Coordinator_\<model-package\>_\<component-name\>. Here it's: 
 
         ./Coordinator_test_bumpBot 

    or 
    
        ./Coordinator_test_collisionDetection

## How to move the Car by using ROS and reading information from the collision Sensor:


1. Start the Carla-Simulator.

2.  Run the following command within the PythonAPI/examples directory: 

        python manual_control.py --rolename=ego_vehicle  
3.  To start the (docker-) container go to the project containing directory and run:  

        docker/run.sh

4.  To move the vehicle by using ROS, open a new terminal and run:  

        docker/compile_exec.sh

5.  You can now read information from the collision sensor by running: 

        docker/open_shell.sh 
        
    and within the shell:
    
        rostopic echo /carla/ego_vehicle/collision

## Correlation with other projects:

This project can be compiled as follows:

1. with the EMAM2Middleware project directly as shown above
2. using the maven-streamtest plugin that also uses the EMAM2Middleware generator.
    - run `mvn clean install -s settings.xml`
 
EMAM2Middleware is build on top of EmbeddedMontiArc

## Sample projects:

[EMAM2Carla](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/carlacomponents/tree/master/EMAM2Carla) is a project where a bumperbot controls a car in the carla-simulator.  
[Autopilot](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/carlacomponents/tree/autopilot-wrapper_experiments) contains code to reuse an autopilot for steering a car in carla.