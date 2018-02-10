#pragma once
#include "IAdapter_ba_system_stopCommQuality_1_.h"
#include "ba_system_stopCommQuality_1_.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
class RosAdapter_ba_system_stopCommQuality_1_: public IAdapter_ba_system_stopCommQuality_1_{
	ba_system_stopCommQuality_1_* component;
	ros::Subscriber _sim_comm_slowdown1Subscriber;
	ros::Publisher _v1_comm_in_slowdown1Publisher;
	
	public:
	RosAdapter_ba_system_stopCommQuality_1_(){
		
	}
	
	void _sim_comm_slowDown1Callback(const std_msgs::Bool::ConstPtr& msg){
		component->in1 = msg->data;
	}
	
	void init(ba_system_stopCommQuality_1_* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_ba_system_stopCommQuality_1__node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_sim_comm_slowdown1Subscriber = node_handle.subscribe("/sim/comm/slowDown1" ,5,&RosAdapter_ba_system_stopCommQuality_1_::_sim_comm_slowDown1Callback, this, ros::TransportHints().tcpNoDelay());
		_v1_comm_in_slowdown1Publisher = node_handle.advertise<std_msgs::Bool>("/v1/comm/in/slowDown1",5);
		ros::spin();
	}
	
	void publish_v1_comm_in_slowdown1Publisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->out1;
		_v1_comm_in_slowdown1Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_v1_comm_in_slowdown1Publisher();
	}
	
};