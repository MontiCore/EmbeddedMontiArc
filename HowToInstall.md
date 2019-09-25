<!-- (c) https://github.com/MontiCore/monticore -->
# How to install everything
- Download Carla (We are using the precompiled version 0.9.5, because this is the latest version that was supported by the ros-bridge at the time) from `https://github.com/carla-simulator/carla/releases/tag/0.9.5`
- Extract it
- If the file carla-0.9.5-py2.7-linux-x86_64.egg is not present inside the PythonAPI folder copy it there from PythonAPI/carla/dist
- Install Docker
- If you want to build the CarlaRosBridge Docker Container yourself you can use the scripts in docker/carla-ros-bridge; note that we use the tag 0.9.5.1 of the carla-ros-bridge as the latest version is not compatible with Carla 0.9.5 anymore.
    You can edit your IP adress in the Dockerfile.

## Steps to build a component:

1.  Move **mw-generator.jar** to the directory of the component you wish to build.
2.  Open shell, cd to the directory of the component and execute: 

        java -jar mw-generator.jar [component].json
        
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
