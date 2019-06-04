package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb {
	tag bumbBot.throttle with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, std_msgs/Float64), msgField = data};
}

