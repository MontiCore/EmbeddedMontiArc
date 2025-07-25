package turtlebot.postprocessor;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.step with RosConnection = {topic=(/post/step, std_msgs/Int32)};
    tag master.cmd_vel_msg with RosConnection = {topic=(/gazebo/step, std_msgs/Float32MultiArray)};
}