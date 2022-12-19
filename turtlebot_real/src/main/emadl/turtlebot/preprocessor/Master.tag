package turtlebot.preprocessor;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.ranges with RosConnection = {topic=(/gazebo/scan, std_msgs/Float32MultiArray)};
    
    tag master.positions with RosConnection = {topic=(/gazebo/position, std_msgs/Float32MultiArray)};
    tag master.state with RosConnection = {topic=(/preprocessor/state, std_msgs/Float32MultiArray)};
    tag master.reward with RosConnection = {topic=(/preprocessor/reward, std_msgs/Float32)};
    tag master.terminal with RosConnection = {topic=(/preprocessor/terminal, std_msgs/Bool)};


    tag master.actionIn with RosConnection = {topic=(/gazebo/actionIn, std_msgs/Int32)};
    tag master.resetState with RosConnection = {topic=(/gazebo/resetState, std_msgs/Bool)};

    tag master.goalReachedBool with RosConnection = {topic=(/gazebo/goalReachedBool, std_msgs/Bool)};
    tag master.goalReached with RosConnection = {topic=(/gazebo/goalReached, std_msgs/Bool)};

}