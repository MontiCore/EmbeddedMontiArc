/* (c) https://github.com/MontiCore/monticore */
package atari.preprocessor;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.screen with RosConnection = {topic=(/gym/state, std_msgs/Float32MultiArray)};
    tag master.gymTerminal with RosConnection = {topic=(/gym/terminal, std_msgs/Bool)};
    tag master.gymReward with RosConnection = {topic=(/gym/reward, std_msgs/Float32)};
    tag master.observation with RosConnection = {topic=(/preprocessor/state, std_msgs/Float32MultiArray)};
    tag master.terminal with RosConnection = {topic=(/preprocessor/terminal, std_msgs/Bool)};
    tag master.reward with RosConnection = {topic=(/preprocessor/reward, std_msgs/Float32)};
}
