ARG CARLA_VERSION='0.9.5'
ARG CARLA_BUILD=''

FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM ros:kinetic-ros-base

RUN apt-get update \
	&& apt-get install -y libpng16-16  ros-kinetic-ackermann-msgs  ros-kinetic-derived-object-msgs ros-kinetic-tf ros-kinetic-cv-bridge --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*

ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
ENV PYTHONPATH=/opt/carla/PythonAPI/carla/dist/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg:/opt/carla/PythonAPI

COPY carla_ackermann_control /opt/carla-ros-bridge/src/carla_ackermann_control
COPY carla_ros_bridge /opt/carla-ros-bridge/src/carla_ros_bridge
COPY carla_manual_control /opt/carla-ros-bridge/src/carla_manual_control
COPY carla_msgs /opt/carla-ros-bridge/src/carla_msgs
COPY carla_ego_vehicle /opt/carla-ros-bridge/src/carla_ego_vehicle/
COPY carla_waypoint_publisher /opt/carla-ros-bridge/src/carla_waypoint_publisher/
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd  /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release install'

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /

# Install Emam & Java
RUN apt-get update \
        && apt-get install -y unzip g++ libopenblas-dev libhdf5-serial-dev cmake maven default-jdk --no-install-recommends \
        && rm -rf /var/lib/apt/lists/*
ENV CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/opt/carla-ros-bridge/devel/share/carla_msgs/cmake"
ENV PATH="/usr/lib/x86_64-linux-gnu/:${PATH}"
ADD https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F18.10.24-armadillo-linux&files=armadillo-8.500.1-linux.zip /root/armadillo.zip
RUN unzip /root/armadillo.zip -d /root/
ENV Armadillo_HOME=/root/armadillo-8.500.1-linux
ENV ROS_HOME=/opt/ros/kinetic
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/kinetic/

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]
