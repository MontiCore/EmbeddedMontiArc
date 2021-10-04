# EMADL Reinforcement Learning Agent for MontiSim

Different EMADL models for Deep Reinforcement Learning with MontiSim.

## Prerequisites


Following tools are needed for this application:

### EMADL
Generation, training and execution were done on Ubuntu 16.04 LTS. Follwoing software is needed (Taken from similar reinforcement learning applications e.g. [TORCS_RF](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/torcs_rf/-/tree/master)):
  
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
- ROS Kinetic (Follow official [installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu))

### MontiSim

Additional installations:

- Maven:
   ```bash
   sudo apt-get install maven
   ```
   
MontiSim:
1. Clone [basic-simulator repository](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator)
2. Install basic-simulator
    ```bash
    cd basic-simulator
    mvn clean install -s settings.xml
    ```
**FOR DEVELOPERS:** When using the standard maven version for Ubuntu 16.04: When installing the simulation project manually, the revision variable which takes care of assigning the correct version number does not work properly. So that the correct versions are used, use the [mavenFix.py](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/additional_files/mavenFix.py) file. Replace the version variable with the correct version. The version is then updated to the standard path to the montisim project ($HOME/.m2/repository/montisim)
    
## Generate,Train and Execute
### Generation
Use the installation script for generation of the python and C++ code:

    ./install.sh

### Training
The training is done within a single script. It starts the ROS master node as well as MontiSim as background processes. As the front process, the training is started. The training automatically resumes the training of the last training state. This can be changed in the script. On script interruption, all background processes are killed. The path to the simulator is stated in the config.sh. The different simulator settings can be changed in the script and can be found in the [Simulation Wiki](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/user-docs/The-basic_simulator)

   ```bash
   ./run_training.sh
   ```

### Execution
With the execution script, a trained agent is executed. For this, the `target/agent/src/de_rwth_montisim_agent_master/cpp/model` folder has to be copied to `target/bin`. Within the model folder, the used parameter file in `model/de.rwth.montisim.agent.network.AutopilotQNet has to be named model_0_newest-0000.params` and the json file has to be named `model_0_newest-symbol.json`.
The script starts the GUI of the simulation. By clicking on the checkboxes at the bottom left, the settings can be chosen before simulation. Then, choose a reinforcement learning scenario. The script executes `./agent -t 100` and therefore executes the agent every 100ms, since one step of the simulator simulates 100ms.

    ./execute.sh

## EMADL Models
There are 3 different Autopilot models that can be used.
1. Only a single vehcile is controlled by the agent
2. 2 vehicles are controlled by the agent, but the agent only calculates the action for one vehicle at the time (Decentralized approach)
3. 2 vehicles are controlled, but the agent receives the combined state of all individual vehicles and outputs all actions at the same time (Centralized approach)

### State and Action Array
Every individual vehicle has a 25-34 dimensional state array, and a 3 dimensinal action array: 

State:
| **Index** | **Name** | **Description** |
|-----------|----------|-----------------|
|0 to 9| trajX | X-Coordinates of the trajectory calculated by the simulator|
|10 to 19 | trajY | Y-Coordinates of the trajectory calculated by the simulator|
|20|trajLength | Length of trajectory |
|21|positionX| X-Coordinate of position of the vehicle|
|22|positionY| Y-Coordinate of position of the vehicle|
|23| currentCompass | Angle, that the vehicle is facing|
|24| currentVelocity | Velocity of the vehicle|
|25 (optional) | currentSpeedLimit| current speed limit of the street|
|26/25 to 34/33 (optional)| Values of different LIDAR sensors|

Action:
| **Index** | **Name** | **Range** | **Description** |
|-----------|----------|-----------|-----------------|
|0|throttle|[-1,1]|Control throttle of the car. -1 equals no throttle, 1 equals full throttle|
|0|braking|[-1,1]|Control brakes of the car. -1 equals no braking, 1 equals full braking|
|0|steering|[-1,1]|Control steering of the car. -1 equals full right steering, 1 equals full left steering|

Since the minimal state is chosen in Autopilot1, the state has dimension 25, and the action dimension 3.

When using the **decentralized** approach, addtional states are appended to the state for every vehicle except the one, which action is currently caluclated. This is done so that vehicle communication is possible (the action does not differ). The additional states are positionX, positionY, currentCompass and currentVelocity. Since the minimal state is chosen in Autopilot2, the state has dimension 25+4=29 and the action has state 3.

When using the **centralized** approach, the states of all vehicles are appended to each other, resulting in one large state. The same is done for the actions. Since the minimal state is chosen in Autopilot3, the state has dimension stateLength(vehicle 1) + stateLength(vehicle 2) = 25 + 25 = 50. The action array has dimension action(vehicle 1) + action(vehicle 2) = 3 + 3 = 6.

## Singularity Container
A Singularity Container can be used for generating, training and executing of MontiSim and its agents. The definition file of that container can be found [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/additional_files/RLContainerNew.def). To use this Container locally, first install singularity ([installation guide](https://sylabs.io/guides/3.7/user-guide/quick_start.html#quick-installation-steps)). Then install the container with the following command (this might take a while): 

    sudo singularity build --sandbox PATH_TO_INSTALLATION_TARGET PATH_TO_DEFINITION_FILE_FOLDER/RLContainerNew.def

Scripts can then be executed within the container by calling:

    singularity exec PATH_TO_CONTAINER PATH_TO_SKRIPT
	
The Singularity Container is also available on the HPC cluster of RWTH Aachen Unisversity under the path `/rwthfs/rz/SW/UTIL.common/singularity/RLContainerNew`.
With this Container, a batch job can be submitted, that executes a script in the Container in the same way, as it is done locally.

**Care**: Due to a bug, the ROS environment variables are not sourced automatically. Hence, following command has to be executed within the script before the start of the ROS master:
```bash
. /opt/ros/kinetic/setup.sh
```

