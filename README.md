Steps to build a component:

* 1.) move **mw-generator.jar** to the directory of the component you wish to build.
* 2.) open shell, cd to the directory of the component and execute: **java -jar mw-generator.jar project.json** (in this case it's called **valid.json** for both components)
* 3.) switch to the target directory (**cd target/**)
* 4.) execute compile.sh (**./compile.sh**)
* 5.) if the generator can't find carla_msgs message type: execute the following command (**export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/path/to/rosbridge/catkin_ws/devel/share/carla_msgs/cmake**)
*       usually the path is something like "**~/carla-ros-bridge/catkin_ws/devel/share/carla_msgs/cmake**"
* 6.) after successfull compiling the generated code switch to install/bin directory (**cd install/bin/**)
* 7.) execute Coordinator_<model-package>_<component-name> (here it's **./Coordinator_test_bumpBot** or **./Coordinator_test_collisionDetection**)
