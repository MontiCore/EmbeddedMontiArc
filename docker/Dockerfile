FROM ubuntu:latest

RUN apt-get update && apt-get -y install \
 mosquitto \
 mosquitto-clients \
 openjdk-8-jdk
ENV JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
RUN apt -y install \
  maven \
  build-essential \
  cmake \
  git
RUN git clone https://github.com/eclipse/paho.mqtt.c.git ; cd paho.mqtt.c ; git checkout v1.2.1 ; cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF  ; cmake --build build/ --target install ; ldconfig
RUN git clone https://github.com/eclipse/paho.mqtt.cpp ; cd paho.mqtt.cpp ; cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF ; cmake --build build/ --target install
RUN rm -rf paho.mqtt.c paho.mqtt.cpp
ADD https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fubuntu%2F18.10.24-armadillo-linux&files=armadillo-8.500.1-linux.zip /root/armadillo.zip
RUN unzip /root/armadillo.zip -d /root/
ENV Armadillo_HOME=/root/armadillo-8.500.1-linux
