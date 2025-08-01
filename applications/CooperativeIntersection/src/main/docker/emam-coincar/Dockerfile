FROM registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2middleware/ema-ros-kinetic
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full ros-kinetic-tf2-geometry-msgs ros-kinetic-geodesy
RUN apt-get update && apt-get install -y python-catkin-tools python-wstool libgeographic-dev libpugixml-dev qt5-default
RUN apt-get update && apt-get install -y wget 
RUN bash -c "mkdir /simulator/; wget -O /simulator/framework.zip \"https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F19.03.07-RosSimulationFramework&files=RosSimulationFramework.zip\"; unzip /simulator/framework.zip -d /simulator/; rm /simulator/framework.zip"
RUN bash -c "source /opt/ros/kinetic/setup.bash; /simulator/RosSimulationFramework/build.sh"
ENV COINCAR_SIM_HOME=/simulator/RosSimulationFramework/
CMD bash -c "rosdep init;source /opt/ros/kinetic/setup.bash; source /simulator/RosSimulationFramework/devel/setup.bash; roslaunch simulation_initialization_ros_tool _whole_framework.launch"
