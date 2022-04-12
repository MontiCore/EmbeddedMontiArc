# ChessAgent
```
        A  B  C  D  E  F  G  H
   8    r  n  b  q  k  b  n  r    8
   7    p  p  p  p  p  p  p  p    7
   6                              6
   5                              5
   4                   P          4
   3                              3
   2    P  P  P  P  P     P  P    2
   1    R  N  B  Q  K  B  N  R    1
        A  B  C  D  E  F  G  H
````
## Compiling chess environment
### Prerequisites
- Java 17 or newer
- Maven 3.8.3 or newer
- *Prolog is needed for the execution of the tests (see [Running chess environment](#running-chess-environment) for installation instructions)* 
### Compilation
```bash
mvn package -s settings.xml
```

## Running chess environment
###  Prerequisites
- SWI-Prolog (Follow [official installation guide](https://www.swi-prolog.org/build/unix.html)) or install on ubuntu:
```bash
apt-get update
apt-get install -y swi-prolog
```
- Roscore
ROS Kinetic (Follow [official installation guide](http://wiki.ros.org/Installation/Ubuntu))

### Running 
- Run environment for training
```bash
roscore #start ROS master
java -cp "target/chess-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.chess.ChessEnv --training
```
- Run environment for gaming
```bash
roscore #start ROS master
java -cp "target/chess-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.chess.ChessEnv --gaming
```
- Run environment for evaluation (in this case for 50 evaluation-episodes)
```bash
roscore #start ROS master
java -cp "target/chess-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.chess.ChessEnv --evaluation 50
```
## Running chess agent
### 1. Install EMADL-Toolchain
  - Install all prerequisites needed for the EMAL toolchain. You can find the prerequisites [here](doc/EMADL_SETUP.md).
### 2. Generate code for training and execution
  - Run 
    ```
    ./install.sh
    ```
### 3. Training
  - Start roscore as well as the game environment
  - Run 
    ```bash
    ./run_training.sh
    ```
### 4. Execution
  - Start roscore as well as the game environment
  - The executable agent is located in the ```bin``` folder. 
  - Execute the agent as follows:
    ```bash
    ./agent -executeOnDemand
    ```