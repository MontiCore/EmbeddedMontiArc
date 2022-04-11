# Setting up a DQN-Agent for a GDL game

## 1. Copy an example agent
- Locate the TicTacToe-Example-Agent at [examples/tictactoe-example-agent](examples/tictactoe-example-agent) and and make a copy of it
### Rename the agent after your game
- Adjust the folder path from ```src/main/emadl/tictactoe/agent``` to ```src/main/emadl/yourgame/agent```.
- Rename the .conf and .emadl in [src/main/emadl/yourgame/agent/network](#) to ```YourGameQNet.conf``` and ```YourGameQnet.emadl``` respectively.

- Adjust the package paths in all emadl and tag files: 
  - ```src/main/emadl/yourgame/agent/network/YourGameQNet.emadl```
  - ```src/main/emadl/yourgame/agent/strategy/GreedyDiscreteStrategy.emadl```
  - ```src/main/emadl/yourgame/agent/Master.emadl```
  - ```src/main/emadl/yourgame/agent/Master.tag```
- Adapt the root model in the following .json:
  - ```config/config.json```

## 2. Modeling the Q-Network
  - Customize the Q-net as desired. CNNArch's specification for modeling the network can be found [here](https://github.com/MontiCore/EmbeddedMontiArc/tree/master/languages/EmbeddedMontiArcDL) or [here](https://github.com/MontiCore/EmbeddedMontiArc/tree/master/languages/CNNArchLang).
  - Adjust the strategy ```src/main/emadl/yourgame/agent/strategy/GreedyDiscreteStrategy.emadl``` accordingly
  - Adjust the inputs and outputs in the ```src/main/emadl/yourgame/agent/Master.emadl```.
  - Change the Ros-topics in the ```src/main/emadl/yourgame/agent/Master.tag```.
## 3. Configure the training
  - Customize the training configuration in ```src/main/emadl/yourgame/agent/network/YourGameQNet.conf```. 
  - Change the name of the agent accordingly.
  - Adapt the ROS-interface
  - An overview of all hyperparameters can be found [here](https://github.com/MontiCore/EmbeddedMontiArc/tree/master/languages/CNNTrainLang).

## 4. Adjust the command line configuration
  - Adjust the following variables in ```config.sh```:
  ```bash
PACKAGE_NAME="yourgame"
NAME_OF_AGENT="YourGameAgent"
NAME_OF_QNET="YourGameQNet"
EMAM_GENERATOR="PATH-TO-GENERATOR"
  ```
## 5. Install EMADL-Toolchain
  - Install all prerequisites needed for the EMAL toolchain. You can find the prerequisites [here](EMADL_SETUP.md).
## 6. Generate code for training and execution
  - First method: Use the .jar that is already included. In that case, leave the variable for the generator untouched in step 4.
  - Second method: Download the [EMAM2Middleware - Library](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware) and compile with Maven. (Follow the instructions on the github-page)
    - Set the EMAM_GENERATOR variable in step 4 to the corresponding artifact.
  - Run 
    ```
    ./install.sh
    ```
## 7. Training
  - Start roscore as well as the game environment [(look here)](GAME_ENV_SETUP.md).
  - Run 
    ```bash
    ./run_training.sh
    ```
## 8. Execution
  - Start roscore as well as the game environment [(look here)](GAME_ENV_SETUP.md).
  - The executable agent is located in the ```bin``` folder. 
  - Execute the agent as follows:
    ```bash
    ./agent -executeOnDemand
    ```