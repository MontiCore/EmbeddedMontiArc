#pragma once
#include "IAdapter_ba_system_stopCommQuality_2_.h"
#include "ba_system_stopCommQuality_2_.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
class RosAdapter_ba_system_stopCommQuality_2_: public IAdapter_ba_system_stopCommQuality_2_{
	ba_system_stopCommQuality_2_* component;
	ros::Subscriber _sim_comm_slowdown2Subscriber;
	ros::Publisher _v2_comm_in_slowdown2Publisher;
	
	public:
	RosAdapter_ba_system_stopCommQuality_2_(){
		
	}
	
	void _sim_comm_slowDown2Callback(const std_msgs::Bool::ConstPtr& msg){
		component->in1 = msg->data;
	}
	
	void init(ba_system_stopCommQuality_2_* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_ba_system_stopCommQuality_2__node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_sim_comm_slowdown2Subscriber = node_handle.subscribe("/sim/comm/slowDown2" ,5,&RosAdapter_ba_system_stopCommQuality_2_::_sim_comm_slowDown2Callback, this, ros::TransportHints().tcpNoDelay());
		_v2_comm_in_slowdown2Publisher = node_handle.advertise<std_msgs::Bool>("/v2/comm/in/slowDown2",5);
		ros::spin();
	}
	
	void publish_v2_comm_in_slowdown2Publisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->out1;
		_v2_comm_in_slowdown2Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_v2_comm_in_slowdown2Publisher();
	}
	
};