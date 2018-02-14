package tests.dist;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags SimpleDist{
tag distComp.sub1.out1 with RosConnection = {topic = (topicIn1, std_msgs/Float64), msgField = data};
tag distComp.sub1.out2 with RosConnection = {topic = (topicIn2, std_msgs/Float64), msgField = data};
tag distComp.sub2.in1 with RosConnection = {topic = (topicIn1, std_msgs/Float64), msgField = data};
tag distComp.sub2.in2 with RosConnection = {topic = (topicIn2, std_msgs/Float64), msgField = data};
}