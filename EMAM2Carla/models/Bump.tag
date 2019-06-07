package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb{
	tag bumpBot.test with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, carla_msgs/CarlaEgoVehicleControl)};
	tag bumpBot.collision_actor_id with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = other_actor_id};
}
