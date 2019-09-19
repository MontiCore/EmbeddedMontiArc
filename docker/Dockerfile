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
  git \
  unzip
RUN git clone https://github.com/eclipse/paho.mqtt.c.git ; cd paho.mqtt.c ; cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF  ; cmake --build build/ --target install
RUN git clone https://github.com/eclipse/paho.mqtt.cpp ; cd paho.mqtt.cpp ; cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF ; cmake --build build/ --target install
RUN rm -rf paho.mqtt.c paho.mqtt.cpp
RUN apt -y install \
  liblapack-dev \
  libblas-dev \
  libboost-dev \
  libarmadillo-dev
