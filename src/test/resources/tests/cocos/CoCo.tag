package tests.cocos;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags CoCo{
//Valid
tag rosToRosComp.subComp1.outPort with
RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};

tag rosToRosComp.subComp2.inPort with
RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};

//Invalid: topic name mismatch
tag topicNameMismatch.subComp1.outPort with
RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};

tag topicNameMismatch.subComp2.inPort with
RosConnection = {topic = (/echo, rosgraph_msgs/Clock), msgField = clock.toSec()};

//Invalid: topic type mismatch
tag topicTypeMismatch.subComp1.outPort with
RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};

tag topicTypeMismatch.subComp2.inPort with
RosConnection = {topic = (/echo, rosgraph_msgs/Clock), msgField = clock.toSec()};

//Invalid: multiple RosConnection on 1 Port
tag portTwoRosConnections.subComp1.outPort with
RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};
tag portTwoRosConnections.subComp1.outPort with
RosConnection = {topic = (/echo, rosgraph_msgs/Clock), msgField = clock.toSec()};

//Invalid: nonRos to Ros
tag noRosToRosComp.subComp2.inPort with
RosConnection = {topic = (/clock, rosgraph_msgs/Clock), msgField = clock.toSec()};


}