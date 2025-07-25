/* (c) https://github.com/MontiCore/monticore */
package pendulum.postprocessor;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.action with RosConnection = {topic=(/postprocessor/step, std_msgs/Float32MultiArray)};
    tag master.postAction with RosConnection = {topic=(/gym/step, std_msgs/Float32MultiArray)};
}
