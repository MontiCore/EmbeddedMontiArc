/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include <ros/ros.h>
#include "ba_intersection_intersectionController.h"
#include <automated_driving_msgs/ObjectStateArray.h>
#include <sim_intersection/BoolWithId.h>
class RosAdapter_ba_intersection_intersectionController{
	ba_intersection_intersectionController component;
	ros::Subscriber _sim_objects_ground_truthSubscriber;
	ros::Publisher _stopPublisher;

public:
	void _sim_objects_ground_truthCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg){
		component.objectIdIn[2] = msg->objects[2].object_id;
		component.objectIdIn[3] = msg->objects[3].object_id;
		component.objectIdIn[0] = msg->objects[0].object_id;
		component.objectIdIn[1] = msg->objects[1].object_id;
	}

	RosAdapter_ba_intersection_intersectionController(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle){
		_sim_objects_ground_truthSubscriber = node_handle.subscribe("/sim/objects_ground_truth", 5, &RosAdapter_ba_intersection_intersectionController::_sim_objects_ground_truthCallback, this, ros::TransportHints().tcpNoDelay());
		_stopPublisher = node_handle.advertise<sim_intersection::BoolWithId>("/stop",5);
	}

	void publish0(){
		sim_intersection::BoolWithId tmpMsg;
		tmpMsg.data = component.stopOut[0];
		tmpMsg.object_id = component.objectIdOut[0];
		_stopPublisher.publish(tmpMsg);
	}

	void publish1(){
		sim_intersection::BoolWithId tmpMsg;
		tmpMsg.data = component.stopOut[3];
		tmpMsg.object_id = component.objectIdOut[3];
		_stopPublisher.publish(tmpMsg);
	}

	void publish2(){
		sim_intersection::BoolWithId tmpMsg;
		tmpMsg.object_id = component.objectIdOut[2];
		tmpMsg.data = component.stopOut[2];
		_stopPublisher.publish(tmpMsg);
	}

	void publish3(){
		sim_intersection::BoolWithId tmpMsg;
		tmpMsg.object_id = component.objectIdOut[1];
		tmpMsg.data = component.stopOut[1];
		_stopPublisher.publish(tmpMsg);
	}

	void tick(){
		component.execute();
		publish0();
		publish1();
		publish2();
		publish3();
	}

};
