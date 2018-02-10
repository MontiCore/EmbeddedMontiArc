#pragma once
#include "IAdapter_ba_system_velocityController_2_.h"
#include "ba_system_velocityController_2_.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
class RosAdapter_ba_system_velocityController_2_: public IAdapter_ba_system_velocityController_2_{
	ba_system_velocityController_2_* component;
	ros::Subscriber ba_system_velocitycontroller_2__holdtimeinSubscriber;
	ros::Subscriber ba_system_velocitycontroller_2__maxvelinSubscriber;
	ros::Subscriber ba_system_velocitycontroller_2__maxaccelinSubscriber;
	ros::Subscriber ba_system_velocitycontroller_2__deltatimeinSubscriber;
	ros::Subscriber _v2_comm_in_slowdown2Subscriber;
	ros::Subscriber ba_system_velocitycontroller_2__resetvelinSubscriber;
	ros::Publisher ba_system_curvel_2_Publisher;
	
	public:
	RosAdapter_ba_system_velocityController_2_(){
		
	}
	
	void _v2_comm_in_slowDown2Callback(const std_msgs::Bool::ConstPtr& msg){
		component->slowDownIn = msg->data;
	}
	
	void ba_system_velocityController_2__deltaTimeInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->deltaTimeIn = msg->data;
	}
	
	void ba_system_velocityController_2__holdTimeInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->holdTimeIn = msg->data;
	}
	
	void ba_system_velocityController_2__maxAccelInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->maxAccelIn = msg->data;
	}
	
	void ba_system_velocityController_2__maxVelInCallback(const std_msgs::Float64::ConstPtr& msg){
		component->maxVelIn = msg->data;
	}
	
	void ba_system_velocityController_2__resetVelInCallback(const std_msgs::Bool::ConstPtr& msg){
		component->resetVelIn = msg->data;
	}
	
	void init(ba_system_velocityController_2_* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_ba_system_velocityController_2__node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_v2_comm_in_slowdown2Subscriber = node_handle.subscribe("/v2/comm/in/slowDown2" ,5,&RosAdapter_ba_system_velocityController_2_::_v2_comm_in_slowDown2Callback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_2__deltatimeinSubscriber = node_handle.subscribe("ba_system_velocityController_2__deltaTimeIn" ,5,&RosAdapter_ba_system_velocityController_2_::ba_system_velocityController_2__deltaTimeInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_2__holdtimeinSubscriber = node_handle.subscribe("ba_system_velocityController_2__holdTimeIn" ,5,&RosAdapter_ba_system_velocityController_2_::ba_system_velocityController_2__holdTimeInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_2__maxaccelinSubscriber = node_handle.subscribe("ba_system_velocityController_2__maxAccelIn" ,5,&RosAdapter_ba_system_velocityController_2_::ba_system_velocityController_2__maxAccelInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_2__maxvelinSubscriber = node_handle.subscribe("ba_system_velocityController_2__maxVelIn" ,5,&RosAdapter_ba_system_velocityController_2_::ba_system_velocityController_2__maxVelInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_velocitycontroller_2__resetvelinSubscriber = node_handle.subscribe("ba_system_velocityController_2__resetVelIn" ,5,&RosAdapter_ba_system_velocityController_2_::ba_system_velocityController_2__resetVelInCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_curvel_2_Publisher = node_handle.advertise<std_msgs::Float64>("ba_system_curVel_2_",5);
		ros::spin();
	}
	
	void publishba_system_curvel_2_Publisher(){
		std_msgs::Float64 tmpMsg;
		tmpMsg.data = component->curVelOut;
		ba_system_curvel_2_Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publishba_system_curvel_2_Publisher();
	}
	
};