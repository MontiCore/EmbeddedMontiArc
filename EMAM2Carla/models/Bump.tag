package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb{
	tag bumpBot.test with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, carla_msgs/CarlaEgoVehicleControl)};
	//tag bumpBot.normal_impulse_x with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse.x};
        //tag bumpBot.normal_impulse_y with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse.y};
        //tag bumpBot.normal_impulse_out with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse};
        //tag bumpBot.normal_impulse with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse};
	//tag bumpBot.normal_impulse_x with RosConnection = {topic = (/emam/collision_direction, geometry_msgs/Vector3), msgField = x};
	tag bumpBot.vehicle_status with RosConnection = {topic = (/carla/ego_vehicle/vehicle_status, carla_msgs/CarlaEgoVehicleStatus)};
	tag bumpBot.collision with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent)};
}
