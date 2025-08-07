/* (c) https://github.com/MontiCore/monticore */
package tictactoe.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.observation with RosConnection = {topic=(/gdl/tictactoe/agent/state, std_msgs/Float32MultiArray)};
    tag master.legal_actions with RosConnection = {topic=(/gdl/tictactoe/agent/legal_actions, std_msgs/Float32MultiArray)};
    tag master.action with RosConnection = {topic=(/gdl/tictactoe/agent/action, std_msgs/Int32)};
}