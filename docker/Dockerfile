FROM ubuntu:latest
RUN apt-get update
RUN apt-get -y install software-properties-common
RUN apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
RUN add-apt-repository ppa:webupd8team/java
RUN apt-get update
RUN apt-get -y install mosquitto
RUN apt-get -y install mosquitto-clients
RUN apt-get -y install openjdk-8-jdk
ENV JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
RUN apt -y install maven
RUN apt-get -y install build-essential
RUN apt-get -y install cmake
RUN apt-get -y install git
RUN git clone https://github.com/eclipse/paho.mqtt.c.git ; cd paho.mqtt.c ; git checkout v1.2.1 ; cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF  ; cmake --build build/ --target install ; ldconfig
RUN git clone https://github.com/eclipse/paho.mqtt.cpp ; cd paho.mqtt.cpp ; cmake -Bbuild -H. -DPAHO_WITH_SSL=OFF ; cmake --build build/ --target install
RUN rm -rf paho.mqtt.c paho.mqtt.cpp
