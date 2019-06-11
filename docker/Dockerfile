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
