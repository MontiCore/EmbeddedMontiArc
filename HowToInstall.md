<!-- (c) https://github.com/MontiCore/monticore -->
# How to install everything
- Download Carla (We are using the precompiled version 0.9.5, because this is the latest version that was supported by the ros-bridge at the time) from `https://github.com/carla-simulator/carla/releases/tag/0.9.5`
- Extract it
- If the file carla-0.9.5-py2.7-linux-x86_64.egg is not present inside the PythonAPI folder copy it there from PythonAPI/carla/dist
- Install Docker
- If you want to build the CarlaRosBridge Docker Container yourself you can use the scripts in docker/carla-ros-bridge; note that we use the tag 0.9.5.1 of the carla-ros-bridge as the latest version is not compatible with Carla 0.9.5 anymore.
    You can edit your IP adress in the Dockerfile.

# How to run it on Linux
## How to execute the code for the bumper bot and read the collision sensor?

- Start Carla by executing: `CarlaUE4.sh`
- Start the ego-vehicle: `docker/egovehicle.sh`      
It executes `python manual_control.py --rolename=ego_vehicle` in PythonAPI/examples to create a vehicle for the ros-bridge.
- To start the container execute inside the project folder:  
`docker/Linux/carla-ros-bridge.sh`

Then execute in a new terminal to compile and execute the generated code for the bumpbot:  
`docker/Linux/bumpbot.sh`

If you want to read additional messages from a rostopic you can execute:  
`docker/Linux/open_shell.sh` and inside the container read the topic with e.g. `rostopic echo /carla/ego_vehicle/collision`

# Running with docker on Windows
I tested this code in a Command Prompt/Power Shell. For the use of other consoles the commands might need to be adjusted for example with winpty.

Similarly to how you run it on Linux, there are shell scripts for Windows: carla-ros-bridge.sh, [component].sh and open_shell.sh. Depending on your system, these might need to be edited.
The scripts worked on the Setup: Windows 10 Home, Docker Toolbox, executed in git-bash.     
To copy the assets into the docker container, I had to have the project folder in an shared-drive (you need to edit the carla-ros-bridge.sh). For me the output in `rostopic echo /carla/ego_vehicle/collision` was with great delay (~1 minute) or none at all, which just might be because of my slow system vs the Carla-simulator.

I do not recommend trying to run everything on Windows.

## Running the code
- In the Docker Settings GUI select the drive where you want to work on as shared drive, else using volumes won't work.

### Updating the IP where carla is running
As we don't have a predefined docker network to the host we have to specify the IP where carla is reachable as follows and rebuild the ros-bridge:

- `sed -i 's/172.17.0.1/<yourIP>/g' /opt/carla-ros-bridge/src/carla_ros_bridge/config/settings.yaml`
- `source /opt/ros/kinetic/setup.bash; cd  /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release install`
- `source /opt/carla-ros-bridge/devel/setup.bash`
- `roslaunch carla_ros_bridge carla_ros_bridge.launch`

### Compile and exec
- This script should work exectly as with Linux:
- `docker exec -it emam2carla bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/emam2carla/EMAM2Carla/ && rm -rf target && java -jar ../mw-generator.jar valid.json && target/compile.sh && target/build/test_bumpBot/coordinator/Coordinator_test_bumpBot'`
