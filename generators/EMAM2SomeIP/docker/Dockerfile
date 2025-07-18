# Use an official Python runtime as a parent image
FROM ubuntu:16.04

# Install Java 8
RUN apt-get update \
    && apt-get -y install openjdk-8-jdk \
    && rm -rf /var/lib/apt/lists/*

ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

ENV PATH $JAVA_HOME/bin:$PATH

#Install cmake, boost, armadillo, maven, git
RUN apt-get update && apt-get -y install \
  build-essential \
  cmake \
  libboost-system1.58-dev libboost-thread1.58-dev libboost-log1.58-dev \
  liblapack-dev libblas-dev libarmadillo-dev \
  maven \
  git

WORKDIR /usr/src/emam2someip

#clone and build vsomeip
RUN git clone https://github.com/GENIVI/vsomeip.git

WORKDIR /usr/src/emam2someip/vsomeip

WORKDIR /usr/src/emam2someip/vsomeip/build

RUN cmake ..

RUN make

RUN make install

WORKDIR /usr/src/emam2someip/vsomeip

RUN rm -rf build

#copy example
COPY . /usr/src/emam2someip

#build example
WORKDIR /usr/src/emam2someip/example/build

RUN cmake ..

RUN make
