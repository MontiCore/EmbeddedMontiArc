/* (c) https://github.com/MontiCore/monticore */
package wrapper;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Wrapper{
	
	tag wrapper.trajectory_length with RosConnection = {topic = (/autopilot/trajectory_length, std_msgs/Int32)};
	tag wrapper.trajectory_x with RosConnection = {topic = (/autopilot/trajectory_x, std_msgs/Float64MultiArray)};
	tag wrapper.trajectory_y with RosConnection = {topic = (/autopilot/trajextory_y, std_msgs/Float64MultiArray)};

	tag wrapper.vehicle_status with RosConnection = {topic = (/carla/ego_vehicle/vehicle_status, carla_msgs/CarlaEgoVehicleStatus)};
	tag wrapper.vehicle_control with RosConnection = {topic = (/carla/ego_vehicle/vehicle_control_cmd, carla_msgs/CarlaEgoVehicleControl)};

	tag calculateWrappedValues.vehicle_status with RosConnection;
	tag calculateWrappedValues.vehicle_control with RosConnection;

	tag autopilot.trajectory_length with RosConnection;
	tag autopilot.trajectory_x with RosConnection;
	tag autopilot.trajectory_y with RosConnection;

	tag wrapper.test with RosConnection = {topic = (/test, std_msgs/Float64), msgField=data};
}
