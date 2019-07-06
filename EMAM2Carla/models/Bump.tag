package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb{
	tag bumpBot.test with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, carla_msgs/CarlaEgoVehicleControl)};
	tag bumpBot.orientation with RosConnection = {topic = (/carla/ego_vehicle/vehicle_status, carla_msgs/CarlaEgoVehicleStatus), msgField = orientation};
	tag bumpBot.collision_normale with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse};
}
