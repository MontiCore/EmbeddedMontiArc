/* (c) https://github.com/MontiCore/monticore */
package bumper;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb{
	tag bumpBot.vehicle_control with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, carla_msgs/CarlaEgoVehicleControl)};
	tag bumpBot.orientation with RosConnection = {topic = (/carla/ego_vehicle/vehicle_status, carla_msgs/CarlaEgoVehicleStatus), msgField = orientation};
	tag bumpBot.collision_normale with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse};
  tag bumpBot.collision_normale_out with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = normal_impulse};
}
