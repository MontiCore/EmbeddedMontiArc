# Use an official Python runtime as a parent image
FROM ubuntu:16.04

RUN apt-get update

RUN apt-get -y install build-essential 

RUN apt-get -y install cmake 

RUN apt-get -y install libboost-system1.58-dev libboost-thread1.58-dev libboost-log1.58-dev

RUN mkdir /usr/src/emam2someip

# Set the working directory to /emam2someip
WORKDIR /usr/src/emam2someip

# Copy the current directory contents into the container at /emam2someip
COPY . /usr/src/emam2someip

RUN mkdir /usr/src/emam2someip/build

WORKDIR /usr/src/emam2someip/build

RUN cmake ..

RUN make

RUN make install

WORKDIR /usr/src/emam2someip

RUN rm -rf build

WORKDIR /usr/src/emam2someip/example/build

RUN cmake ..

RUN make
