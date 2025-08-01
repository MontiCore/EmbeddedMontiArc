/* (c) https://github.com/MontiCore/monticore */
package tests.structs;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags ArrayHandling{
tag arrayHandlingComp.in1 with RosConnection = {topic = (/name1, std_msgs/Float64MultiArray), msgField = data[:]};
tag arrayHandlingComp.in2 with RosConnection = {topic = (/name2, std_msgs/ByteMultiArray)};
tag arrayHandlingComp.in3 with RosConnection = {topic = (/name3, nav_msgs/Path), msgField = poses[:].pose.orientation.x};
tag arrayHandlingComp.in4 with RosConnection = {topic = (/name4, nav_msgs/Path), msgField = poses[2:6].pose.orientation.x};
tag arrayHandlingComp.out1 with RosConnection = {topic = (/name5, std_msgs/ByteMultiArray)};
tag arrayHandlingComp.out2 with RosConnection = {topic = (/name6, std_msgs/Int32MultiArray)};
}
