#pragma once
#include "IAdapter_cheetah_master.h"
#include "cheetah_master.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class RosAdapter_cheetah_master: public IAdapter_cheetah_master{
	
	cheetah_master* component;
	ros::Subscriber _gym_stateSubscriber;
	ros::Publisher _gym_stepPublisher;
	
	public:
	RosAdapter_cheetah_master(){
		
	}
	
	void init(cheetah_master* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_cheetah_master_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		_gym_stateSubscriber = node_handle.subscribe("/gym/state", 5, &RosAdapter_cheetah_master::_gym_stateCallback, this, ros::TransportHints().tcpNoDelay());
		
		_gym_stepPublisher = node_handle.advertise<std_msgs::Float32MultiArray>("/gym/step",5);
		
		ros::spin();
	}
	
	void _gym_stateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
		int counter = 0;
		for(int i0 = 0; i0 < 26; i0++){
			(component->state)(i0) = msg->data[counter];
			counter++;
		}
		
	}
	
	void publish_gym_stepPublisher(){
		std_msgs::Float32MultiArray tmpMsg;
		tmpMsg.data.resize(6);
		int counter = 0;
		for(int i0 = 0; i0 < 6; i0++){
			tmpMsg.data[counter] = (component->action)(i0);
			counter++;
		}
		
		_gym_stepPublisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_gym_stepPublisher();
	}
	
};