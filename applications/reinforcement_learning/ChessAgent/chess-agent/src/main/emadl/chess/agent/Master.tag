/* (c) https://github.com/MontiCore/monticore */
package chess.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.observation with RosConnection = {topic=(/gdl/chess/agent/state, std_msgs/Float32MultiArray)};
    tag master.legal_actions with RosConnection = {topic=(/gdl/chess/agent/legal_actions, std_msgs/Float32MultiArray)};
    tag master.action with RosConnection = {topic=(/gdl/chess/agent/action, std_msgs/Int32)};
}