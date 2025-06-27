/* (c) https://github.com/MontiCore/monticore */
package tests.a;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags EchoRos2 {
    tag compB.rosIn with RosConnection = {topic = (/clock, std_msgs/msg/Float64), msgField = data};
    tag compB.rosOut with RosConnection = {topic = (/echo, std_msgs/msg/Float64), msgField = data};
}
