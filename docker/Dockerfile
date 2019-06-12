# Use an official Python runtime as a parent image
FROM ubuntu:16.04

RUN apt-get update

RUN apt-get -y install build-essential 

RUN apt-get -y install cmake 

RUN apt-get -y install libboost-system1.58-dev libboost-thread1.58-dev libboost-log1.58-dev

RUN apt-get update \
    && apt-get -y install openjdk-8-jre \
    && rm -rf /var/lib/apt/lists/*

ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

ENV PATH $JAVA_HOME/bin:$PATH

RUN apt-get update

RUN apt-get -y install maven

RUN apt-get -y install git

RUN mkdir /usr/src/emam2someip

WORKDIR /usr/src/emam2someip

RUN git clone https://github.com/GENIVI/vsomeip.git

#copy example
COPY . /usr/src/emam2someip

WORKDIR /usr/src/emam2someip/vsomeip

RUN mkdir /usr/src/emam2someip/vsomeip/build

WORKDIR /usr/src/emam2someip/vsomeip/build

RUN cmake ..

RUN make

RUN make install

WORKDIR /usr/src/emam2someip/vsomeip

RUN rm -rf build

WORKDIR /usr/src/emam2someip/example/build

RUN cmake ..

RUN make


