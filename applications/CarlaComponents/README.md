<!-- (c) https://github.com/MontiCore/monticore -->
## Dependences 
[Here](HowToInstall.md) is how to install everything (and how to run it manually).

# How to move the Car by using ROS and reading information from the collision Sensor:
## How to run it on Linux

1. Start the Carla-Simulator (`CarlaUE4.sh`)

2. Start the ego-vehicle: 

        docker/egovehicle.sh

    It executes `python manual_control.py --rolename=ego_vehicle` in PythonAPI/examples to create a vehicle for the ros-bridge.
3.  To start the (docker-) container and carla-ros-bridge:  

        docker/Linux/carla-ros-bridge.sh

4.  To move the vehicle by using ROS, open a new terminal and run:  

        docker/Linux/bumpbot.sh
    
    This builds the bumpbot component and runs it.
5.  You can now read information from the collision sensor by running: 

        docker/Linux/open_shell.sh 
        
    and within the shell:
    
        rostopic echo /carla/ego_vehicle/collision

## Running with docker on Windows
I tested this code in a Command Prompt/Power Shell. For the use of other consoles the commands might need to be adjusted for example with winpty.

Similarly to how you run it on Linux, there are shell scripts for Windows: carla-ros-bridge.sh, [component].sh and open_shell.sh. Depending on your system, these might need to be edited.
The scripts worked on the Setup: Windows 10 Home, Docker Toolbox, executed in git-bash.     
To copy the assets into the docker container, I had to have the project folder in an shared-drive (you need to edit the carla-ros-bridge.sh). For me the output in `rostopic echo /carla/ego_vehicle/collision` was with great delay (~1 minute) or none at all, which just might be because of my slow system vs the Carla-simulator.

I do not recommend trying to run everything on Windows.

## Correlation with other projects:

This project can be compiled as follows:

1. with the EMAM2Middleware project directly as shown above
2. using the maven-streamtest plugin that also uses the EMAM2Middleware generator.
    - run `mvn clean install -s settings.xml`
 
EMAM2Middleware is build on top of EmbeddedMontiArc

## Sample projects:

[Bumpbot](Bumpbot) is a project where a bumperbot controls a car in the carla-simulator.  
[Autopilot](Autopilot) contains code to reuse an autopilot for steering a car in carla.
