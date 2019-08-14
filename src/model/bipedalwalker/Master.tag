package bipedalwalker;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.state with RosConnection = {topic=(/gym/state, std_msgs/Float32MultiArray)};
    tag master.action with RosConnection = {topic=(/gym/step, std_msgs/Float32MultiArray)};
}