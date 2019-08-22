/* (c) https://github.com/MontiCore/monticore */
package test;
conforms to de.monticore.lang.monticar.generator.roscpp.RosToEmamTagSchema;

tags Bumb{
	tag collisionDetection.other_actor_id with RosConnection = {topic = (/carla/ego_vehicle/collision, carla_msgs/CarlaCollisionEvent), msgField = other_actor_id};
	tag collisionDetection.stdout with RosConnection = {topic = (/echo, std_msgs/Int64), msgField = data};
}

