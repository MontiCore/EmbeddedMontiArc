package de.rwth.montisim.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
	tag master.state with RosConnection = {topic=(sim/state, std_msgs/Float32MultiArray)};
	tag master.action with RosConnection = {topic=(/sim/step, std_msgs/Float32MultiArray)};
	tag master.terminated with RosConnection = {topic=(/sim/terminal, std_msgs/Bool)};
	tag master.terminate with RosConnection = {topic=(/sim/reset, std_msgs/Bool)};
}