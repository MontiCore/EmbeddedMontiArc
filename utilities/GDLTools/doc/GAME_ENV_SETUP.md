# Setting up a game environment for a GDL game

## 1. Copy an example environment
- Copy the exemplary TicTacToe environment located at [examples/tictactoe-example-environment](examples/tictactoe-example-environment)

## 2. Modeling a game in GDL
- Locate ```TicTacToeExample.gdl``` at [examples/tictactoe-example-environment/src/main/resources/gdl/games/TicTacToeExample.gdl](examples/tictactoe-example-environment/src/main/resources/gdl/games/TicTacToeExample.gdl) and rename it to ```YourGame.gdl```
- Model your game and save it as ```YourGame.gdl```

## 3. Setting up the environment
The game environment is set up via a Java class.
### Renaming
- Locate [TicTacToeEnvironmentExample.java](examples/tictactoe-example-environment/src/main/java/de/gdl/rl/environment/games/tictactoe/TicTacToeEnvironmentExample.java) and rename it to ```YourGameEnvironment.java```
(as well as all occurrences of TicTacToeEnvironmentExample in the file)
-  adapt artifactId and groupId in [pom.xml](examples/tictactoe-example-environment/pom.xml) accordingly
### Loading GDL model
```java
String getPathToGdlModel() {...}
```
- Adapt this method of ```YourGameEnvironment.java``` to return path to ```src/main/resources/gdl/games/YourGame.gdl``` 
### Mapping state to float vector
- For a DQN agent to learn a GDL game, the state returned by the GDL interpreter as a list of tuples must be mapped to a float array. The mapping can be done depending on a role, for example for symmetric games the state can be inverted. Implement the following method in ```YourGameEnvironment.java```: 
```java
public float[] getStateAsFloatRepresentation(String gdlRoleName) {...}
```
### Numbering of all actions
- Think about a numbering for all possible actions that can occur in a game for a specific role. Implement the following methods in ```YourGameEnvironment.java``` to create a bidirectional mapping between actions and their corresponding numbers:
```java
public String getMoveStringFromAction(int action, String gdlRoleName) {...}
public int getActionFromMoveString(Command move) {...}
```
- Specify the number of possible actions for each role by implementing the following method in ```YourGameEnvironment.java```:
```java
public int getNumberOfActionsForRole(String roleName) {...}
```
### Assignment of roles to agents
There are three different types of agents that can be added to the environment: RosTrainingAgent, RosAgent and LocalAgent. 
- RosTrainingAgents are agents that are in training and should be connected via ROS-interface. They have ROS topics for transferring state, actions, end of game / episode, reward and for resetting. In the constructor of ```YourGameEnvironment.java``` they are defined as shown below:
```java
RosTrainingAgent exampleTrainingAgent = new RosTrainingAgent()
    .withName("Example Training Agent")
    .withType("DQN")
    .withGdlRoleNames(new String[]{"x", "o"})
    .withGameOverForIllegalActions(true)
    .withCurrentGdlRoleName("x")
    .withStateTopic("/gdl/tictactoe/trainingAgent/state")
    .withActionTopic("/gdl/tictactoe/trainingAgent/action")
    .withTerminalTopic("/gdl/tictactoe/trainingAgent/terminal")
    .withRewardTopic("/gdl/tictactoe/trainingAgent/reward")
    .withResetTopic("/gdl/tictactoe/trainingAgent/reset");
```
- RosAgents are agents that are in execution. For example, they can act as opponents.
Besides a ROS-topic for state and actions, these have a topic for legal moves.
```java
RosAgent exampleAgent = new RosAgent()
    .withName("Agent")
    .withType("DQN")
    .withGdlRoleNames(new String[]{"x", "o"})
    .withStateTopic("/gdl/tictactoe/agent/state")
    .withLegalActionsTopic("/gdl/tictactoe/agent/legal_actions")
    .withActionTopic("/gdl/tictactoe/agent/action");
```
In addition, LocalAgents can be added. For example, these can serve as a baseline for evaluation. The ```LocalAgent``` class can be used for subclassing.
```java
ExampleLocalAgent exampleLocalAgent = new ExampleLocalAgent()
```
### Adding agents
- Agents can be added to either a training or game configuration. The training configuration is used to train agents. A new game is started when all training agents send a signal via the reset topic. In the game configuration, a user can specify via CLI which roles are played manually. 
- Agents can be added to the training configuration in the constructor of ```YourGameEnvironment.java``` as follows:
```java
this.addToTrainingConfiguration(exampleTrainingAgent);
```
- Agents can be added to the gaming configuration in the constructor of ```YourGameEnvironment.java``` as follows:
```java
this.addToGamingConfiguration(exampleAgent);
```
### *Reward calculation [optional]*
- By default, the goal from the GDL model is taken as the reward. If the calculation of the reward needs to be adjusted, this can be done individually for each role implementing the following method in ```YourGameEnvironment.java```: 
```java
public float calculateRewardFromGoals(List<List<String>> goals, String gdlRoleName) {...}
```
### *Role selection in case of ambiguity [optional]*
- In some games, it may not be dictated which role performs the next move. By default, the selection is random, otherwise the following method can be used for customization:

```java
public String whichRoleShouldBeNext(List<String> availableRoles) {...}
```
### *Alterations during the execution [optional]*
- Sometimes it is helpful to change the configuration during execution. For example, a different role can be assigned to a training agent. After each episode, the following method is called for this purpose, which can be used as an event accordingly:
```java
public void onNextEpisode() { ... }
```
### Auxiliary functions
- Get the current state:
```java
public List<List<String>> getCurrentState()
```
- Get all legal moves for a role:
```java
public List<String> getLegalMovesForRole(String gdlRoleName)
```
- Get the last executed move:
```java
public String getLastExecutedMove()
```
- Indicates whether the last executed move was illegal:
```java
public boolean wasLastMoveIllegal()
```
- Indicates whether the game is over:
```java
public boolean isTerminal()
```
## 4. Testing the environment
- The mapping of the state can be reviewed with the following method. For this, the state is outputted before and after the conversion. It can be specified whether a certain number of random moves should be performed to obtain a sequence of conversions. 
```java
YourGameEnvironment.stateMappingTest(new YourGameEnvironment(), <<numberOfRandonEpisodes>>);
```
- The bidirectional mapping can be tested for all specified agents of all configurations and all possible roles. Each action encoded by a number is first mapped to the corresponding move and back. Subsequently, it is checked whether input and output matches.
```java
 YourGameEnvironment.bidirectionalActionMappingTest(new YourGameEnvironment());
```

## 5. Compiling the environment
### Prerequisites
- Java 17 or newer
- Maven 3.8.3 or newer
- *Prolog is needed for the execution of the tests (see 6. for installation instructions)* 
### Compilation
```bash
mvn package -s settings.xml
```

## 6. Running the environment
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
java -cp "target/yourgame-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.yourgame.YourGameEnvironment --training
```
- Run environment for gaming
```bash
roscore #start ROS master
java -cp "target/yourgame-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.yourgame.YourGameEnvironment --gaming
```
- Run environment for evaluation (in this case for 50 evaluation-episodes)
```bash
roscore #start ROS master
java -cp "target/yourgame-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.yourgame.YourGameEnvironment --gaming --evaluation 50
```