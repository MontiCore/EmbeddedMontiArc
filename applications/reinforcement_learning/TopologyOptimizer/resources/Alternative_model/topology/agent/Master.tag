package topology.agent;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Master {
    tag master.state with RosConnection = {topic=(/topo/state, std_msgs/Float32MultiArray)};
    tag master.action with RosConnection = {topic=(/topo/step, std_msgs/Float32MultiArray)};
    tag master.terminated with RosConnection = {topic=(/topo/terminal, std_msgs/Bool)};
    tag master.terminate with RosConnection = {topic=(/topo/reset, std_msgs/Bool)};
}
