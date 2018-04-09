#pragma once
#include "IAdapter_ba_system_velocityController_1_.h"
#include "ba_system_velocityController_1_.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
class RosAdapter_ba_system_velocityController_1_: public IAdapter_ba_system_velocityController_1_{
	ba_system_velocityController_1_* component;
	ros::Subscriber ba_system_velocitycontroller_1__holdtimeinSubscriber;
	ros::Subscriber ba_system_velocitycontroller_1__maxvelinSubscriber;
	ros::Subscriber ba_system_velocitycontroller_1__maxaccelinSubscriber;
	ros::Subscriber ba_system_velocitycontroller_1__deltatimeinSubscriber;
	ros::Subscriber _v1_comm_in_slowdown1Subscriber;
	ros::Subscriber ba_system_velocitycontroller_1__resetvelinSubscriber;
	ros::Publisher ba_system_curvel_1_Publisher;
	
	public:
	RosAdapter_ba_system_velocityController_1_(){
		
	}
	
	void _v1_comm_in_slowDown1Callback(const std_msgs::Bool::ConstPtr& msg){
		component->slowDownIn = msg->data;
	}
	
	void ba_system_velocityController_1__deltaTimeInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->deltaTimeIn = msg->data;
	}
	
	void ba_system_velocityController_1__holdTimeInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->holdTimeIn = msg->data;
	}
	
	void ba_system_velocityController_1__maxAccelInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->maxAccelIn = msg->data;
	}
	
	void ba_system_velocityController_1__maxVelInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->maxVelIn = msg->data;
	}
	
	void ba_system_velocityController_1__resetVelInCallback(const std_msgs::Bool::ConstPtr& msg){
		component->resetVelIn = msg->data;
	}
	
	void init(ba_system_velocityController_1_* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_ba_system_velocityController_1__node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_v1_comm_in_slowdown1Subscriber = node_handle.subscribe("/v1/comm/in/slowDown1" ,5,&RosAdapter_ba_system_velocityController_1_::_v1_comm_in_slowDown1Callback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_1__deltatimeinSubscriber = node_handle.subscribe("ba_system_velocityController_1__deltaTimeIn" ,5,&RosAdapter_ba_system_velocityController_1_::ba_system_velocityController_1__deltaTimeInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_1__holdtimeinSubscriber = node_handle.subscribe("ba_system_velocityController_1__holdTimeIn" ,5,&RosAdapter_ba_system_velocityController_1_::ba_system_velocityController_1__holdTimeInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_1__maxaccelinSubscriber = node_handle.subscribe("ba_system_velocityController_1__maxAccelIn" ,5,&RosAdapter_ba_system_velocityController_1_::ba_system_velocityController_1__maxAccelInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_1__maxvelinSubscriber = node_handle.subscribe("ba_system_velocityController_1__maxVelIn" ,5,&RosAdapter_ba_system_velocityController_1_::ba_system_velocityController_1__maxVelInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_1__resetvelinSubscriber = node_handle.subscribe("ba_system_velocityController_1__resetVelIn" ,5,&RosAdapter_ba_system_velocityController_1_::ba_system_velocityController_1__resetVelInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_curvel_1_Publisher = node_handle.advertise<std_msgs::Float64>("ba_system_curVel_1_",5);
		ros::spin();
	}
	
	void publishba_system_curvel_1_Publisher(){
		std_msgs::Float64 tmpMsg;
		tmpMsg.data = component->curVelOut;
		ba_system_curvel_1_Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publishba_system_curvel_1_Publisher();
	}
	
};