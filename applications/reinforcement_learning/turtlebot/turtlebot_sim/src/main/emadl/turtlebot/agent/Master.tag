package turtlebot.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.state with RosConncetion = {topic=(/preprocessor/state, std_msgs/Float32MultiArray)};
    tag master.action with RosConncetion = {topic=(/gazebo/step, std_msgs/Int32)};
}