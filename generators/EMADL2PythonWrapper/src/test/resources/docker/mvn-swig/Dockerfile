FROM maven:3-jdk-8

# install packages
RUN apt-get update && apt-get install -q -y \
        wget python gcc \
        build-essential cmake \
        liblapack-dev libblas-dev libboost-dev libarmadillo-dev \
        make cmake python2.7 python-dev python-pip python-numpy swig openjdk-8-jre libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --user numpy
