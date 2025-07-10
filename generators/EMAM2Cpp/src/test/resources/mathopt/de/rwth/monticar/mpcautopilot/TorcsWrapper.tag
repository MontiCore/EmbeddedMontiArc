/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticar.mpcautopilot;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags TorcsWrapper {
    tag torcsWrapper.state with RosConnection = {topic=(/torcs/state, std_msgs/Float32MultiArray)};
    tag torcsWrapper.action with RosConnection = {topic=(/torcs/step, std_msgs/Float32MultiArray)};
}
