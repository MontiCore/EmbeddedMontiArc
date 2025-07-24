/*
 * From EMADL2CPP for MXNet.
 */
PACKAGE de.monticore.lang.monticar.sol.tests.environment

COMPONENT DOCKERFILE SecondEnvironment

INSTALL "git"
INSTALL "wget"
INSTALL "python"
INSTALL "gcc"
INSTALL "build-essential"
INSTALL "cmake"
INSTALL "liblapack-dev"
INSTALL "libarmadillo-dev"

RUN "rm -rf /var/lib/apt/lists/*"

RUN "git clone https://github.com/apache/incubator-mxnet.git mxnet-source"

RUN "python get-pip.py"

RUN "pip install mxnet h5py"
