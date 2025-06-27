<!-- (c) https://github.com/MontiCore/monticore -->
![pipeline](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNTrainLang/badges/master/build.svg)
![coverage](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/CNNTrainLang/badges/master/coverage.svg)

# !!This project is obsolete !! Use ConfLang instead !!

# CNNTrain

CNNTrain is a domain specific language for describing training parameters of a feedforward neural network. CNNTrain files must have .cnnt extension. Training configuration starts with a `configuration` word, followed by the configuration name and a list of parameters.  The available parameters are batch size, number of epochs, loading previous checkpoint as well as an optimizer with its parameters. All these parameters are optional.

An example of a config:

```
configuration FullConfig{
     num_epoch : 5
     batch_size : 100
     load_checkpoint: true
     optimizer:rmsprop{
         learning_rate:0.001
         weight_decay:0.01
         learning_rate_decay:0.9
         learning_rate_policy:step
         step_size:1000
         rescale_grad:1.1
         clip_gradient:10
         gamma1:0.9
         gamma2:0.9
         epsilon:0.000001
         centered:true
         clip_weights:10
     }
}
```

See CNNTrain.mc4 for full grammar definition.

Using CNNTrainGenerator class, a Python file can be generated, which looks as following (for an example above):

 ```python
batch_size = 100,
num_epoch = 5,
load_checkpoint = True,
optimizer = 'rmsprop',
optimizer_params = {
    'epsilon': 1.0E-6,
    'weight_decay': 0.01,
    'rescale_grad': 1.1,
    'centered': True,
    'clip_gradient': 10.0,
    'gamma2': 0.9,
    'gamma1': 0.9,
    'learning_rate_policy': 'step',
    'clip_weights': 10.0,
    'learning_rate': 0.001,
    'learning_rate_decay': 0.9,
    'step_size': 1000}
 ```

## Reinforcement Learning

CNNTrain can be used to describe training parameters for supervised learning methods as well as for reinforcement learning methods. If reinforcement learning is selected, the network is trained with the Deep-Q-Network algorithm (Mnih et. al. in Playing Atari with Deep Reinforcement Learning).

An example of a supervised learning configuration can be seen above. The following is an example configuration for reinforcement learning:

```CNNTrainLang
configuration ReinforcementConfig {
    learning_method : reinforcement

    agent_name : "reinforcement-agent"

    environment : gym { name:"CartPole-v1" }

    context : cpu

    num_episodes : 300
    num_max_steps : 9999
    discount_factor : 0.998
    target_score : 1000
    training_interval : 10

    loss : huber

    use_fix_target_network : true
    target_network_update_interval : 100

    use_double_dqn : true

    replay_memory : buffer{
        memory_size : 1000000
        sample_size : 64
    }

    action_selection : epsgreedy{
        epsilon : 1.0
        min_epsilon : 0.01
        epsilon_decay_method: linear
        epsilon_decay : 0.0001
    }

    optimizer : rmsprop{
        learning_rate : 0.001
        learning_rate_minimum : 0.00001
        weight_decay : 0.01
        learning_rate_decay : 0.9
        learning_rate_policy : step
        step_size : 1000
        rescale_grad : 1.1
        clip_gradient : 10
        gamma1 : 0.9
        gamma2 : 0.9
        epsilon : 0.000001
        centered : true
        clip_weights : 10
    }
}
```

### Available Parameters for Reinforcement Learning

| Parameter  |  Value | Default | Required | Algorithm | Description |
|------------|--------|---------|----------|-----------|-------------|
|learning_method| reinforcement,supervised | supervised | No | All | Determines that this CNNTrain configuration is a reinforcement or supervised learning configuration |
| rl_algorithm | ddpg-algorithm, dqn-algorithm, td3-algorithm | dqn-algorithm | No | All | Determines the RL algorithm that is used to train the agent
| agent_name | String | "agent" | No | All | Names the agent (e.g. for logging output) |
|environment | gym, ros_interface | Yes | / | All | If *ros_interface* is selected, then the agent and the environment communicates via [ROS](http://www.ros.org/). The gym environment comes with a set of environments which are listed [here](https://gym.openai.com/) |
| context    | cpu, gpu | cpu | No | All | Determines whether the GPU is used during training or the CPU |
| num_episodes | Integer | 50 | No | All | Number of episodes the agent is trained. An episode is a full passing of a game from an initial state to a terminal state.|
| num_max_steps | Integer | 99999 | No | All | Number of steps within an episodes before the environment is forced to reset the state (e.g. to avoid a state in which the agent is stuck) |
|discount_factor | Float | 0.9 | No | All | Discount factor |
| target_score | Float | None | No | All | If set, the agent stops the training when the average score of the last 100 episodes is greater than the target score. |
| training_interval | Integer | 1 | No | All | Number of steps between two trainings |
| loss | l2, l1, softmax_cross_entropy, sigmoid_cross_entropy, huber | l2 | No | DQN | Selects the loss function
| use_fix_target_network | bool | false | No | DQN | If set, an extra network with fixed parameters is used to estimate the Q values |
| target_network_update_interval | Integer | / | DQN | Yes, if fixed target network is true | If *use_fix_target_network* is set, it determines the number of steps after the target network is updated (Minh et. al. "Human Level Control through Deep Reinforcement Learning")|
| use_double_dqn | bool | false | No | If set, two value functions are used to determine the action values (Hasselt et. al. "Deep Reinforcement Learning with Double Q Learning") |
| replay_memory | buffer, online, combined | buffer | No | All | Determines the behaviour of the replay memory |
| strategy | epsgreedy, ornstein_uhlenbeck | epsgreedy (discrete), ornstein_uhlenbeck (continuous) | No | All |  Determines the action selection policy during the training |
| reward_function | Full name of an EMAM component | / | Yes, if *ros_interface* is selected as the environment  and no reward topic is given | All | The EMAM component that is used to calculate the reward. It must have two inputs, one for the current state and one boolean input that determines if the current state is terminal. It must also have exactly one output which represents the reward. |
critic | Full name of architecture definition | / | Yes, if DDPG or TD3 is selected | DDPG, TD3 | The architecture definition which specifies the architecture of the critic network |
soft_target_update_rate | Float | 0.001 | No | DDPG, TD3 | Determines the update rate of the critic and actor target network |
actor_optimizer | See supervised learning | adam with LR .0001 | No | DDPG, TD3 | Determines the optimizer parameters of the actor network |
critic_optimizer | See supervised learning | adam with LR .001 | No | DDPG, TD3 | Determines the optimizer parameters of the critic network |
| start_training_at | Integer | 0 | No | All | Determines at which episode the training starts |
| evaluation_samples | Integer | 100 | No | All | Determines how many epsiodes are run when evaluating the network |
| policy_noise | Float | 0.1 | No | TD3 | Determines the standard deviation of the noise that is added to the actions predicted by the target actor network when calculating the targets.
| noise_clip | Float | 0.5 | No | TD3 | Sets the upper and lower limit of the policy noise
policy_delay | Integer | 2 | No | TD3 | Every policy_delay of steps, the actor network and targets are updated.

#### Environment

##### Option: ros_interface

If selected, the communication between the environment and the agent is done via ROS. Additional parameters:

- **state_topic**: Topic on which the state is published
- **action_topic**: Topic on which the action is published
- **reset_topic**: Topic on which the reset command is published
- **terminal_state_topic**: Topic on which the terminal flag is published

##### Option: gym

The gym environment comes with a set of environments which are listed [here](https://gym.openai.com/). Additional parameters:

- **name**: Name (see https://gym.openai.com/) of the environment

#### Replay Buffer

Different buffer behaviour can be selected for the training. For more information about the buffer behaviour see "A deeper look at Experience Replay" by Zhang, Sutton

##### Option: buffer

A simple buffer in which stores the SARS (**S**tate, **A**ction, **R**eward, next **S**tate) tuples. Additional parameters:

- **memory_size**: Determines the size of the buffer
- **sample_size**: Number of samples that are used for each training step

##### Option: online

No buffer is used. Only the current SARS tuple is used for taining.

##### Option: combined

Combination of *online* and *buffer*. Both the current SARS tuple as well as a sample from the buffer are used for each training step. Parameters are the same as *buffer*.

### Strategy

Determines the behaviour when selecting an action based on the values.

#### Option: epsgreedy

This strategy is only available for discrete problems. It selects an action based on Epsilon-Greedy-Policy. This means, based on epsilon, either a random action is choosen or an action with the highest Q-value. Additional parameters:

- **epsilon**: Probability of choosing an action randomly
- **epsilon_decay_method**: Method which determines how epsilon decreases after each step. Can be *linear* for linear decrease or *no* for no decrease.
- **epsilon_decay_start**: Number of Episodes after the decay of epsilon starts
- **epsilon_decay**: The actual decay of epsilon after each step.
- **min_epsilon**: After *min_epsilon* is reached, epsilon is not decreased further.
- **epsilon_decay_per_step**:Expects either true or false. If true, the decay will be performed for each step the agent executes instead of performing the decay after each episode. The default value is false


#### Option: ornstein_uhlenbeck
This strategy is only available for continuous problems. The action is selected based on the actor network. Based on the current epsilon, noise is added based on the [Ornstein-Uhlenbeck](https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process) process. Additional parameters:

All epsilon parameters from epsgreedy strategy can be used. Additionally, **mu**, **theta**, and **sigma** needs to be specified. For each action output you can specify the corresponding value with a tuple-style notation: `(x,y,z)`

Example: Given an actor network with action output of shape (3,), we can write

```EMADL
    strategy: ornstein_uhlenbeck{
        ...
        mu: (0.0, 0.1, 0.3)
        theta: (0.5, 0.0, 0.8)
        sigma: (0.3, 0.6, -0.9)
    }
```

to specify the parameters for each place.

### Option: gaussian
This strategy is also only available for continuous problems. If this strat- egy is selected, uncorrelated Gaussian noise with zero mean is added to the current policy action selection. This strategy provides the same parameters as the epsgreedy option and the parameter **noise_variance** that determines the variance of the noise.

## Generation

To execute generation in your project, use the following code to generate a separate Config file:

```java
import de.monticore.lang.monticar.cnntrain.generator.CNNTrainGenerator;
...
CNNTrainGenerator cnnTrainGenerator =  new CNNTrainGenerator();
Path modelPath = Paths.get("path/to/cnnt/file");
cnnTrainGenerator.generate(modelPath, cnnt_filename);
```

Use the following code to get file contents as a map ( `fileContents.getValue()` contains the generated code):

```java
import de.monticore.lang.monticar.cnntrain.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
...
CNNTrainGenerator cnnTrainGenerator =  new CNNTrainGenerator();
ModelPath mp = new ModelPath(Paths.get("path/to/cnnt/file"));
GlobalScope trainScope = new GlobalScope(mp, new CNNTrainLanguage());
Map.Entry<String, String> fileContents = cnnTrainGenerator.generateFileContent( trainScope, cnnt_filename );
```

 CNNTrain can be used together with [CNNArch](https://github.com/EmbeddedMontiArc/CNNArchLang) language, which describes architecture of a NN. [EmbeddedMontiArcDL](https://github.com/EmbeddedMontiArc/EmbeddedMontiArcDL) uses both languages.
