# EMADL Reinforcement Learning Agent for MontiSim

Framework for Reinforcement Learning Agents for [MontiSim](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation), along with pretrained experiments.

## Requirements
All installations and trainings were tested on a fresh Ubuntu 20.04 LTS.

It is recommended to use either the CPU singularity container [RLContainerCPU.def](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/-/blob/main/additional_files/RLContainerCPU.def) or the GPU singularity container [RLContainerGPU.def](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/-/blob/main/additional_files/RLContainerGPU.def) for local trainings.

For installing and using singularity containers, visit [this section](#singularity-container).

Alternatively, the scripts [installation_script_CPU.sh](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/-/blob/main/additional_files/installation_script_CPU.sh) and [installation_script_CPU.sh](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/-/blob/main/additional_files/installation_script_CPU.sh) can be used to locally install the requirements.

> :warning: Using GPUs during training requires a CUDA compatible GPU, and the installation of CUDA and CuDNN. The CUDA version to be installed depends on GPU, OS Version, and MXNET version. As of now, MXNET version 1.9.1 was used. The CuDNN version depends on the CUDA version.
https://mxnet.apache.org/versions/1.9.1/get_started?platform=linux&language=python&processor=gpu&environ=pip&

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
3. Clone [simulation repository](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation)
4. Install ROSInterface (Found in the simulation project)
5. Copy libROSInterface.so into install directory of basic-simulator project


### EMADL
In order to generate the agent code from [CNNArchLang](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNArchLang) and [CNNTrainLang](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNTrainLang) files, the EMADL pipeline will have to be built. For this, clone and build the following repositories:
 - [CNNArch2X](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/cnnarch2x)
 - [CNNArch2Gluon](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/CNNArch2Gluon)
 - [EMADL2CPP](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMADL2CPP)
 - [EMAM2Middleware](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware)

Finally, under `target/` in the EMAM2Middleware project, a jar file named `embedded-montiarc-math-middleware-generator-X.Y.Z-SNAPSHOT-jar-with-dependencies.jar` will be generated. This jar file will be used to generate the agent code, and **must be placed** in the `coopmontisimautopilot/Autopilot/bin` directory.

## Background
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
|26/25 to 34/33 (optional)| lidarSensors| Values of different LIDAR sensors|

Action:
| **Index** | **Name** | **Range** | **Description** |
|-----------|----------|-----------|-----------------|
|0|throttle|[-1,1]|Control throttle of the car. -1 equals no throttle, 1 equals full throttle|
|1|braking|[-1,1]|Control brakes of the car. -1 equals no braking, 1 equals full braking|
|2|steering|[-1,1]|Control steering of the car. -1 equals full right steering, 1 equals full left steering|

Since the minimal state is chosen in Autopilot1, the state has dimension 25, and the action dimension 3.

When using the **decentralized** approach, addtional states are appended to the state for every vehicle except the one, which action is currently caluclated. This is done so that vehicle communication is possible (the action does not differ). The additional states are positionX, positionY, currentCompass and currentVelocity. Since the minimal state is chosen in Autopilot2, the state has dimension 25+4=29 and the action has state 3.

When using the **centralized** approach, the states of all vehicles are appended to each other, resulting in one large state. The same is done for the actions. Since the minimal state is chosen in Autopilot3, the state has dimension stateLength(vehicle 1) + stateLength(vehicle 2) = 25 + 25 = 50. The action array has dimension action(vehicle 1) + action(vehicle 2) = 3 + 3 = 6.

### Self-play approach
A new self-play option was added to MontiSim for the training with multiple vehicles. Thereby one vehicle is trained via EMADL, while all other vehicles are simulated by a self-play agent which is executed simultaneously. Therefore the agent binary generated in the beginning of the training is executed with the latest version of the EMADL agents policy. To allow updating of the agent over the course of the training the network weights of the self-play agent are updated after every snapshot. Thus the **update_agent.sh** gets executed which handles the process and restarts the self-play agent with the updated network weights. This script is executed by the toolchain automatically.
Depending on the trained number of vehicles the execution interval of the self-play agent in the **run_training.sh** and **update_agent.sh** script can be adapted. It makes sense to increase the execution interval for an increasing number of vehicles since the self-play agent then sends its actions to MontiSim faster.
There exist three different ways the EMADL trained vehicle switches during the training:
1. No Update at all (nU).
2. Update after every simulation step (aS).
3. Update after every episode (aE).

These options can be selected in the GUI and CLI (nU, aS, aE) of the Simulator where an switch after every episode provided the best results.

## Generate,Train and Execute
After placing the `embedded-montiarc-math-middleware-generator-X.Y.Z-SNAPSHOT-jar-with-dependencies.jar` file in the `coopmontisimautopilot/Autopilot/bin` directory, adjust `config.sh` to match the middleware version.
Furthermore, the variable `SELF_PLAY_AGENT_EXECUTION_INTERVAL_TRAINING` can be adjusted to speed up training. During training, one should see close to no messages `Cant keep up` in the output. The lower the value though, the better. Minimize it to your system but no messages should appear.
The variable `SELF_PLAY_AGENT_EXECUTION_INTERVAL_RUNNING` should be exactly the same duration (in ms) as one simulation step is long. Default value is 100 ms. If the simulation step size is changed (in the scenario file), this variable should be adjusted accordingly.

Finally, the paths of the basic_simulator and others might have to be adjusted in `config.sh`.

### Training
Use the installation script for generation of the python and C++ code:
```bash
./install.sh
```

The training is done within a single script. It starts the ROS master node as well as MontiSim as background processes. As the front process, the training is started. The different simulator settings can be changed in the script and can be found in the [Simulation Wiki](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/user-docs/The-basic_simulator). Currently, the simulator is started in the decentralized approach, changing the trained vehicle after every episode. For more information, I refer to [the background section](#background)

When training multiple vehicles, the self-play approach is used by default. In order to active it, set `self_play: yes` in the CNNTrainLang file. In the training scripts, use the `-sp` flag:
```bash
./run_training.sh # use this for single vehicle training
./initial_run_training.sh -sp # use this for multiple vehicle training
```

Alternatively, the installation and training can be called from a single script:
```bash
./install_and_run_training.sh # use this for single vehicle training
./install_and_run_training.sh -sp # use this for single vehicle training
```

When using self-play, the **initial_run_training.sh** script is called (in the background). This is executed for the first training run after installation when using the self-play approach. The reason for that is, that the initial policy needs to be created to be used by the self-play agent in the beginning of the training.

After training, the best networks parameters along with every snapshot are saved under `target/agent/src/de_rwth_montisim_agent_master/cpp/model/AutopilotAgent/*/`.

Training Data can be visualized using the tools in the `tools` directory. View its README for more information.

### Execution
In order to execute the trained network, the parameter files have to be moved to `target/bin/model/de.rwth.montisim.agent.network.AutopilotQNet/` and the files have to be renamed to `model_0_newest-0000.params` and `model_0_newest-symbol.json`.
Alternatively, the process of copying and renaming the best network can be automated by using the `-auto` flag:
```bash
./execute.sh # execute using manually copied files
./execute.sh -auto # automatically execute best network
```

This script then starts the MontiSim in RL execution mode. By clicking on the checkboxes at the bottom left, the settings can be chosen before simulation. Most importantly, the option `Use decentralized reinforcement learning` has to be checked.
Then, a scenario file (**starting with `rl_`**) can be chosen from the `basic_simulator/install/scenarios/` directory.

### Save configurations
To easily save previous Autopilot trainings when making changes in the training configuration the save_autopilot.sh script can be used.

    ./save_autopilot.sh

## Singularity Container
A Singularity Container can be used for generating, training and executing of MontiSim and its agents. The definition file of a CPU container can be found at [RLContainerCPU.def](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/-/blob/main/additional_files/RLContainerCPU.def) or the GPU container at [RLContainerGPU.def](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/applications/reinforcement_learning/coopmontisimautopilot/-/blob/main/additional_files/RLContainerGPU.def).
To use this Container locally, first install singularity ([installation guide](https://sylabs.io/guides/3.7/user-guide/quick_start.html#quick-installation-steps)). Then install the container with the following command (this might take a while):

```bash
sudo singularity build --sandbox PATH_TO_INSTALLATION_TARGET PATH_TO_DEFINITION_FILE_FOLDER/RLContainerNew.def
```

Scripts can then be executed within the CPU container by calling:
```bash
singularity exec PATH_TO_CONTAINER PATH_TO_SKRIPT
```
... or within the GPU container by calling:
```bash
singularity exec --nv PATH_TO_CONTAINER PATH_TO_SKRIPT
```

The Singularity Containers are also available on the HPC cluster of RWTH Aachen University under the path `/rwthfs/rz/SW/UTIL.common/singularity/RLContainerCPU` and `/rwthfs/rz/SW/UTIL.common/singularity/RLContainerGPU`.
With this Container, a batch job can be submitted, that executes a script in the Container in the same way, as it is done locally.
For training on the Cluster, [SLURM Jobs](https://help.itc.rwth-aachen.de/service/rhr4fjjutttf/article/13ace46cfbb84e92a64c1361e0e4c104/) have to be submitted. For this, modify the `cluster_training.sh` scripts to your needs:
 - CPU or GPU Training?
 - Resource Specification
 - Thesis Project ID
 - Self-Play (`-sp`) or Single Vehicle?

> :warning: **BUG:** GPU training on the cluster is currently only working for training **without** self-play. When training on the cluster a self-play agent using GPU, MXNET doesn't detect the GPU.
