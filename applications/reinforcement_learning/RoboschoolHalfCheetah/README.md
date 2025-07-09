<!-- (c) https://github.com/MontiCore/monticore -->
# Reinforcement Learning Agent for RoboschoolHalfCheetah-v1

EMADL models of a DDPG and TD3 Agent for the [OpenAi Gym environment](https://gym.openai.com/envs/RoboschoolHalfCheetah-v1/) `RoboschoolHalfCheetah-v1`. 
This project includes training configurations for both DDPG and TD3.

This project only includes the EMADL model. Unfortunately, the roboschool environment of OpenAI Gym is only available for Python 3. Since rospy is only available for Python 2.7, we are not able to run the executable model and the ros-gym environment. However, we modified the generated reinforcment learning trainer such that it is executable with Python 3. Therefore, you can run the training of the model.

## Prerequisites

In order to run this application you need the following tools:

### General

Generation, training, and execution were tested on Ubuntu 16.04 LTS. The generation, training, and execution of the model requires the following tools and packages:  
- Java 8, Build Tools (make, cmake, gcc), Git, Python 2.7, Python 3, pip, numpy, SWIG:

    ```bash
    sudo apt install openjdk-8-jre gcc make cmake git python2.7 python-dev python-numpy swig libboost-all-dev curl python3
    ```
- Python pip:
    ```bash
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    python get-pip.py --user
    ```

- Python 3 packages for numpy, h5py, pyprind, matplotlib:

    ```bash
    pip install --user h5py numpy pyprind matplotlib
    ```

    

- MXNet C++ Language Bindings (Follow official [installation guide](https://mxnet.incubator.apache.org/versions/master/install/ubuntu_setup.html))
- MXNet for Python 3 (Follow official [installation guide](https://mxnet.incubator.apache.org/versions/master/install/index.html?platform=Linux&language=Python&processor=CPU))
- Armadillo >= 9.400.3 (Follow official [installation guide](http://arma.sourceforge.net/download.html))
- ROS Kinetic (Follow official [installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu))
- OpenAI-Gym for Python 3: Follow official [installation guide](https://github.com/openai/gym#installation)
- OpenAI Roboschool: Follow official [installation guide](https://github.com/openai/roboschool)


## Training

You can run the training by running:
```bash
./run_training
```

## Troubleshooting
- **MXNET Library not found:** During the building of the executable model, we require the MXNet libraries. Ensure that  `/usr/include` and `/usr/lib` include the MXNet library files. You can add the library files by doing the following: 
    ```bash
    cd /usr/include
    sudo ln -s path/to/incubator-mxnet/include/mxnet .
    cd /usr/lib
    sudo cp ~/incubator-mxnet/build/libmxnet.so .
    ```
- **fatal error: numpy/arrayobject.h: No such file or directory** The numpy include files are not included in the global `/usr/include` directory. Usually, the missing files can be found in the directory `/usr/local/lib/python2.7/dist-packages/numpy/core/include/numpy`. If not, you can run a python script which outputs the location:
    ```python
    import numpy
    numpy.get_include()
    ```
    After that you can copy the files to the global include directory:
    ```bash
    cp -r /usr/local/lib/python2.7/dist-packages/numpy/core/include/numpy /usr/local/include
    ```
