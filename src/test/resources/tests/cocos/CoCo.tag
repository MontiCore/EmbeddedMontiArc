package tests.cocos;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags CoCo{
//Valid
tag rosToRosComp.subComp1.outPort with RosConnection =
{topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};
tag rosToRosComp.subComp2.inPort with RosConnection =
{topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};

//Invalid: Non matching topic

//Invalid: nonRos to Ros

}