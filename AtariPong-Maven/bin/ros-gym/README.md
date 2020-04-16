<!-- (c) https://github.com/MontiCore/monticore -->
# ROS Communicator for Gym Environemnts

The RosGym environment creates a ROS interface for the OpenAI environments. Given an environment string, which can be found on the [OpenAI Gym webpage](\url{https://gym.openai.com}), we can interact with the environment via ROS. The application sends all states via the topic */gym/state*. The terminal information of a state is published via the topic */gym/terminal*. All rewards are sent via the topic */gym/reward*. A new game can be started by sending a boolean value via */gym/reset*. An agent can provide an action to the environment by using the */gym/step* topic. We can launch the RosGym environment with the CLI.

## Usage

Launch the program with ```python launcher.py -e EVNVIRONMENT_STRING [OPTIONS]```

All options:

|Option | Shortcut |Description|
|-------|----------|-----------|
| --environment *str* (required) | -e *str*| *str* is the name of the Open Ai environment |
| --verbose | -v | show logging information |
| --quiet | -q | show no logging information |
| --render *x*| -r *x* | Render the environment every *x* steps
| --play | -p | Starts the environment in play mode (one game is played), otherwise it is started in training mode |
| --continuous | -c | Required flag if the action space of the environment is continuous. If not set, the environment assumes that the action space is discrete. |
| --video | -m | Records a video of the game played. |
| --eval | | Evaluation mode in which 100 games are played. The games are automatically restarted after they finished. After the 100 games, the environment will output statistical data about the games played. |
