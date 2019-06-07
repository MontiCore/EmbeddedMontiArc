Steps to build a component:

* 1.) move **mw-generator.jar** to the directory of the component you wish to build.
* 2.) open shell, cd to the directory of the component and execute: **java -jar mw-generator.jar project.json** (in this case it's called **valid.json** for both components)
* 3.) switch to the target directory (**cd target/**)
* 4.) execute compile.sh (**./compile.sh**)
* 5.) if the generator can't find carla_msgs message type: execute the following command and retry step 4
*       export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/path/to/rosbridge/catkin_ws/devel/share/carla_msgs/cmake
*       usually the path is something like "~/carla-ros-bridge/catkin_ws/devel/share/carla_msgs/cmake"
* 6.) after successfull compiling the generated code switch to install/bin directory (**cd install/bin/**)
* 7.) execute Coordinator_<model-package>_<component-name> (here it's **./Coordinator_test_bumpBot** or **./Coordinator_test_collisionDetection**)

## Wie Bringe ich das auto zum Fahren und lese den collision sensor aus?

Carla Starten  
Unter PythonAPI/examples: `python manual_control.py --rolename=ego_vehicle`  
Zum Starten des Containers in den Ordner des Projektes (hier ein ausgechecktes Verzeichnis BumbBot_test) gehen:  
`docker run -it --name emam2carla --rm -v $(pwd):/usr/src/project registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/carlacomponents/emam-carla-ros-bridge`

Dann in einem neuen Terminal um das Auto fahren zu lassen:  
`docker exec -it emam2carla /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/project/ &&  java -jar ../mw-generator.jar valid.json && target/compile.sh && build/test_bumpBot/coordinator/Coordinator_test_bumpBot'`

Um den Collision Sensor auszulesen in einem neuen Terminal:  
`docker exec -it emam2carla /bin/bash -c 'source /opt/ros/kinetic/setup.bash && source opt/carla-ros-bridge/devel/setup.bash && rostopic echo /carla/ego_vehicle/collision'`