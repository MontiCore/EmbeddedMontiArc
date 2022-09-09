package de.rwth.montisim.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.state with RosConnection = {topic=(sim/state2, std_msgs/Float32MultiArray)};
    tag master.action with RosConnection = {topic=(/sim/step2, std_msgs/Float32MultiArray)};
    tag master.terminated with RosConnection = {topic=(/sim/terminal2, std_msgs/Bool)};
    tag master.terminate with RosConnection = {topic=(/sim/reset2, std_msgs/Bool)};
}
