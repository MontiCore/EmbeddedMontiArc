/*
 * From EMADL2CPP for Caffe2.
 */
PACKAGE de.monticore.lang.monticar.sol.tests.environment

COMPONENT DOCKERFILE FirstEnvironment

INSTALL "build-essential", "cmake", "git", "libgoogle-glog-dev", "libgtest-dev", "libiomp-dev", "libleveldb-dev"
INSTALL "liblmdb-dev", "libopencv-dev", "libprotobuf-dev", "python-dev", "python-pip"

RUN "pip install future numpy protobuf"

ENV "NAME" "World"
ENV "PYTHONPATH" = "/usr/local"
ENV "PYTHONPATH" = "${PYTHONPATH}:/pytorch/build"
ENV "PYTHONPATH" = "${PYTHONPATH}:/usr/bin/python"
ENV "LD_LIBRARY_PATH" = "/usr/local/lib"

RUN "git clone --recursive https://github.com/pytorch/pytorch.git"
