<!-- (c) https://github.com/MontiCore/monticore -->
# Reinforcement Learning Agent for BipedalWalker-v2

EMADL models of a DDPG and TD3 Agent for the [OpenAi Gym environment](https://gym.openai.com/envs/BipedalWalker-v2/) `BipedalWalker-v2`. Based on the models, code is generated that trains on the mentioned environment.
We provide a prebuild version with already trained parameters.
This project includes training configurations and prebuilds for both DDPG and TD3.

## Prerequisites

In order to run this application you need the following tools:

### General

Generation, training, and execution were tested on Ubuntu 16.04 LTS. The generation, training, and execution of the model requires the following tools and packages:
  
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

- MXNet C++ Language Bindings (Follow official [installation guide](https://mxnet.incubator.apache.org/versions/master/install/ubuntu_setup.html))
- MXNet for Python (Follow official [installation guide](https://mxnet.incubator.apache.org/versions/master/install/index.html?platform=Linux&language=Python&processor=CPU))
- Armadillo >= 9.400.3 (Follow official [installation guide](http://arma.sourceforge.net/download.html))
- ROS Kinetic (Follow official [installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu))

- 
- OpenAI-Gym: Follow official [installation guide](https://github.com/openai/gym#installation)

Note that you require the [Box2D](https://github.com/openai/gym/blob/master/docs/environments.md) package of OpenAI-Gym.


## Generate and Train

**Attention:** Check the training context in the training configuration `src/model/bipedalwalker/agent/WalkerActor.cnnt`. Only use the context "gpu" if you have installed the CUDA packages and the corresponding version of MXNet. Otherwise choose the context "cpu".

1) The `bin` folder already includes a prebuild of the middleware generator. We can start the generation by running the install script.
    ```bash
    ./install.sh
    ```
    The install script will generate the Python Reinforcement Learning trainer and the C++ files for the executable model into the `target` folder. Furthermore, the install script will automatically build the executable model. The binary can be found in `target/bin/agent`.

2) Start the training by running
    ```bash
    ./run_training
    ```

After the training you will find the trained weight parameters of the agent in the folder `target/agent/src/root_model_name/cpp/model`. Move the model folder to the binaries of the executable model in `target/bin`. Be sure that the names of the files are always `model_0_newest-symbol.json` for the symbol file and `model_0_newest-0000.params` for the weight parameters. Otherwise, the predictor in the executable model will not find the files. You can run the agent with:
```bash
./agent -t 100
```

During the training, the agent will make snapshots of the weights. You can command the trainer to make snapshots every fixed-number of episodes in the training configuration. You will find all snapshots, log files, and statistical files in the folder `target/agent/src/root_model_name/cpp/model/AgentName`.
There are three files that are created during a snapshot:
- The symbol file `AGENT_NAME-epXXX-symbol.json`.
- The weight parameters for the Python Gluon networks `AGENT_NAME-epXXX.params`
- The weight parameters for the C++ Predictor `AGENT_NAME-epXXX-0000.params`

You can also interrupt the training at any time with SIGINT (Ctrl+c). Then, the agent will receive an interrupt signal. After the current running episode, the agent stores its session state (network weights, parameters, and content of the replay memory) into a file. The current weights and symbol file can be found in the folder `target/agent/src/root_model_name/cpp/model/AgentName`. If you want to continue the session, just rerun the above described steps. If the trainer can find the session file, it will ask if you want to continue from the last episode.

## Run Prebuild
We provide prebuilds of a TD3 and DDPG agent with trained weights in the `prebuild` folder. For example, run the following command to test the TD3 agent.
```bash
cd prebuild/td3
./run.sh
```
This will start the master ROS node for the communication, the ros-gym environment, and the agent. Then, the agent plays 100 games in the environment and returns its scores.

The ros-gym environment has other options for playing the environment. For example, you can render the game screen. Look at bin/ros-gym for more details.

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
