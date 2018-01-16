#!/bin/bash
sudo apt-get install libblas-dev liblapack-dev
wget http://sourceforge.net/projects/arma/files/armadillo-8.300.3.tar.xz
tar -xvf armadillo-8.300.3.tar.xz
cd armadillo-8.300.3
cmake .
make
sudo make install