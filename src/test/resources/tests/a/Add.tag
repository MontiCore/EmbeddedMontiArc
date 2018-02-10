package tests.a;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Add {
tag addComp.in1 with RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};
tag addComp.out1 with RosConnection = {topic = (/echo, automated_driving_msgs/StampedFloat64), msgField = data};
}