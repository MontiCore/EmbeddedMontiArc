/* (c) https://github.com/MontiCore/monticore */
package pendulum.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.state with RosConnection = {topic=(/gym/state, std_msgs/Float32MultiArray)};
    tag master.action with RosConnection = {topic=(/postprocessor/step, std_msgs/Float32MultiArray)};
}
