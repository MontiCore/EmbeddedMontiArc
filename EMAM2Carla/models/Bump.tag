package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb{
	tag bumpBot.test with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, carla_msgs/CarlaEgoVehicleControl)};
	tag bumpBot.lane_invasion with RosConnection = {topic = (/carla/ego_vehicle/lane_invasion, carla_msgs/CarlaLaneInvasionEvent), msgField = msg.crossed_lane_markings(1)};
	tag bumpBot.collision_actor_id with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = msg.other_actor_id};
}

