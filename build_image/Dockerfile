#
# (c) https://github.com/MontiCore/monticore
#

FROM debian

ENV DEBIAN_FRONTEND noninteractive


RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    openjdk-11-jdk maven \
    && rm -rf /var/lib/apt/lists/*
