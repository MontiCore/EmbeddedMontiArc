#pragma once
#include "IAdapter_ba_system_collisionDetection.h"
#include "ba_system_collisionDetection.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <struct_msgs/ba_struct_Rectangle.h>
class RosAdapter_ba_system_collisionDetection: public IAdapter_ba_system_collisionDetection{
	const int n = 2;
	const int x = 1;
	ba_system_collisionDetection* component;
	ros::Subscriber ba_system_collisiondetection_hulls_1_Subscriber;
	ros::Subscriber ba_system_collisiondetection_hulls_2_Subscriber;
	ros::Publisher ba_system_collisionoutPublisher;
	
	public:
	RosAdapter_ba_system_collisionDetection(){
		
	}
	
	void ba_system_collisionDetection_hulls_1_Callback(const struct_msgs::ba_struct_Rectangle::ConstPtr& msg){
		component->hulls[0].pointA.posX = msg->pointA.posX;
		component->hulls[0].pointA.posY = msg->pointA.posY;
		component->hulls[0].pointB.posX = msg->pointB.posX;
		component->hulls[0].pointB.posY = msg->pointB.posY;
		component->hulls[0].pointC.posX = msg->pointC.posX;
		component->hulls[0].pointC.posY = msg->pointC.posY;
		component->hulls[0].pointD.posX = msg->pointD.posX;
		component->hulls[0].pointD.posY = msg->pointD.posY;
	}
	
	void ba_system_collisionDetection_hulls_2_Callback(const struct_msgs::ba_struct_Rectangle::ConstPtr& msg){
		component->hulls[1].pointA.posX = msg->pointA.posX;
		component->hulls[1].pointA.posY = msg->pointA.posY;
		component->hulls[1].pointB.posX = msg->pointB.posX;
		component->hulls[1].pointB.posY = msg->pointB.posY;
		component->hulls[1].pointC.posX = msg->pointC.posX;
		component->hulls[1].pointC.posY = msg->pointC.posY;
		component->hulls[1].pointD.posX = msg->pointD.posX;
		component->hulls[1].pointD.posY = msg->pointD.posY;
	}
	
	void init(ba_system_collisionDetection* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_ba_system_collisionDetection_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		ba_system_collisiondetection_hulls_1_Subscriber = node_handle.subscribe("ba_system_collisionDetection_hulls_1_" ,5,&RosAdapter_ba_system_collisionDetection::ba_system_collisionDetection_hulls_1_Callback, this, ros::TransportHints().tcpNoDelay());
		ba_system_collisiondetection_hulls_2_Subscriber = node_handle.subscribe("ba_system_collisionDetection_hulls_2_" ,5,&RosAdapter_ba_system_collisionDetection::ba_system_collisionDetection_hulls_2_Callback, this, ros::TransportHints().tcpNoDelay());
		ba_system_collisionoutPublisher = node_handle.advertise<std_msgs::Bool>("ba_system_collisionOut",5);
		ros::spin();
	}
	
	void publishba_system_collisionoutPublisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->collision;
		ba_system_collisionoutPublisher.publish(tmpMsg);
	}
	
	void tick(){
		publishba_system_collisionoutPublisher();
	}
	
};