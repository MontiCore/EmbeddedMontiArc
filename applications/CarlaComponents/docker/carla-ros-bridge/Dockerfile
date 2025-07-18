ARG CARLA_VERSION='0.9.5'
ARG CARLA_BUILD=''

FROM carlasim/carla:$CARLA_VERSION$CARLA_BUILD as carla

FROM ros:kinetic-ros-base

RUN apt-get update -qq \
	&& apt-get -qq install -y --allow-unauthenticated libpng16-16  ros-kinetic-ackermann-msgs  ros-kinetic-derived-object-msgs ros-kinetic-tf ros-kinetic-cv-bridge --no-install-recommends

ARG CARLA_VERSION

COPY --from=carla --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
ENV PYTHONPATH=/opt/carla/PythonAPI/carla/dist/carla-$CARLA_VERSION-py2.7-linux-x86_64.egg:/opt/carla/PythonAPI

RUN git clone --branch 0.9.5.1 https://github.com/carla-simulator/ros-bridge.git /opt/carla-ros-bridge/git
RUN mkdir -p /opt/carla-ros-bridge/src
RUN cp -r /opt/carla-ros-bridge/git/carla_ackermann_control /opt/carla-ros-bridge/src/carla_ackermann_control
RUN cp -r /opt/carla-ros-bridge/git/carla_ros_bridge /opt/carla-ros-bridge/src/carla_ros_bridge
RUN cp -r /opt/carla-ros-bridge/git/carla_manual_control /opt/carla-ros-bridge/src/carla_manual_control
RUN cp -r /opt/carla-ros-bridge/git/carla_msgs /opt/carla-ros-bridge/src/carla_msgs
RUN cp -r /opt/carla-ros-bridge/git/carla_ego_vehicle /opt/carla-ros-bridge/src/carla_ego_vehicle
RUN cp -r /opt/carla-ros-bridge/git/carla_waypoint_publisher /opt/carla-ros-bridge/src/carla_waypoint_publisher

# REPLACE WITH YOUR IP if not used on linux with default docker network
RUN sed -i 's/localhost/172.17.0.1/g' /opt/carla-ros-bridge/src/carla_ros_bridge/config/settings.yaml
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd  /opt/carla-ros-bridge; catkin_make -DCMAKE_BUILD_TYPE=Release install'

# replace entrypoint
RUN cp -r /opt/carla-ros-bridge/git/docker/content/ros_entrypoint.sh /

# Install Emam & Java
RUN apt-get install -y -qq --allow-unauthenticated unzip g++ libopenblas-dev libhdf5-serial-dev cmake maven default-jdk --no-install-recommends \
        && rm -rf /var/lib/apt/lists/*
ENV CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/opt/carla-ros-bridge/devel/share/carla_msgs/cmake"
ENV PATH="/usr/lib/x86_64-linux-gnu/:${PATH}"
ADD https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F18.10.24-armadillo-linux&files=armadillo-8.500.1-linux.zip /root/armadillo.zip
RUN unzip /root/armadillo.zip -d /root/
ENV Armadillo_HOME=/root/armadillo-8.500.1-linux
ENV ROS_HOME=/opt/ros/kinetic
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/kinetic/

CMD ["roslaunch", "carla_ros_bridge", "carla_ros_bridge.launch"]
