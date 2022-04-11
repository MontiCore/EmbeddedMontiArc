# Setup of the EMADL toolchain

Code for training, which is specified via the configuration files, and execution of agents modeled with EMADL is generated with the EMAM2Middleware generator. The following prerequisites are necessary to be able to generate and execute the code:

## Prerequisites
  
- Java 8, Build Tools (make, cmake, gcc), Git, Python 2.7, pip, numpy, SWIG:

    ```bash
    sudo apt install openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl
    ```
- Python pip:

    ```bash
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    python get-pip.py --user
    ```

- Python packages for numpy, h5py, pyprind, matplotlib:

    ```bash
    pip install --user h5py numpy pyprind matplotlib
    ```

- MXNet C++ Language Bindings:
    ```
    git clone --recursive https://github.com/apache/incubator-mxnet.git mxnet
    cd mxnet && git checkout tags/1.5.0 && git submodule update --recursive --init
    mkdir build && cd build && cmake -DUSE_CPP_PACKAGE=1 -DUSE_CUDA=0 -GNinja .. && ninja -v
    cd .. && sudo cp -r include/mxnet ../usr/include/mxnet && sudo cp -r cpp-package/include/mxnet-cpp ../usr/include && sudo cp -r 3rdparty/tvm/nnvm/include/nnvm ../usr/include && cp -r 3rdparty/dmlc-core/include/dmlc ../usr/include
    
- MXNet for Python:

    ```bash
    pip install mxnet
    ```
- Armadillo >= 9.400.3 (Follow official [installation guide](http://arma.sourceforge.net/download.html))
- ROS >= Kinetic (Follow official [installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu))
