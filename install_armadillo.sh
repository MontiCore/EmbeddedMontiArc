#!/bin/bash
sudo apt-get install libblas-dev liblapack-dev
cd ./src/integration-test/resources
tar -xJf armadillo-8.300.3.tar.xz
cd armadillo-8.300.3
cmake .
make
sudo make install