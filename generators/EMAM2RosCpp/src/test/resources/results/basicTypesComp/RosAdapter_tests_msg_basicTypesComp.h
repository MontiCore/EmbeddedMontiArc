/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_msg_basicTypesComp.h"
#include "tests_msg_basicTypesComp.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

class RosAdapter_tests_msg_basicTypesComp: public IAdapter_tests_msg_basicTypesComp{
	
	tests_msg_basicTypesComp* component;
	
	bool topic7Callback_wasCalled;
	bool topic8Callback_wasCalled;
	bool topic9Callback_wasCalled;
	
	ros::Subscriber topic7Subscriber;
	ros::Subscriber topic8Subscriber;
	ros::Subscriber topic9Subscriber;
	ros::Publisher topic7Publisher;
	ros::Publisher topic8Publisher;
	ros::Publisher topic9Publisher;
	
	public:
	RosAdapter_tests_msg_basicTypesComp(){
		
	}
	
	void init(tests_msg_basicTypesComp* comp){
		this->component = comp;
		topic7Callback_wasCalled = false;
		topic8Callback_wasCalled = false;
		topic9Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_msg_basicTypesComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		topic7Subscriber = node_handle.subscribe("topic7", 5, &RosAdapter_tests_msg_basicTypesComp::topic7Callback, this, ros::TransportHints().tcpNoDelay());
		topic8Subscriber = node_handle.subscribe("topic8", 5, &RosAdapter_tests_msg_basicTypesComp::topic8Callback, this, ros::TransportHints().tcpNoDelay());
		topic9Subscriber = node_handle.subscribe("topic9", 5, &RosAdapter_tests_msg_basicTypesComp::topic9Callback, this, ros::TransportHints().tcpNoDelay());
		
		topic7Publisher = node_handle.advertise<std_msgs::Float64>("topic7",5);
		topic8Publisher = node_handle.advertise<std_msgs::Int32>("topic8",5);
		topic9Publisher = node_handle.advertise<std_msgs::Bool>("topic9",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && topic7Callback_wasCalled && topic8Callback_wasCalled && topic9Callback_wasCalled;
	}
	
	void topic7Callback(const std_msgs::Float64::ConstPtr& msg){
		
		component->inQ = msg->data;
		
		topic7Callback_wasCalled = true;
	}
	void topic8Callback(const std_msgs::Int32::ConstPtr& msg){
		
		component->inZ = msg->data;
		
		topic8Callback_wasCalled = true;
	}
	void topic9Callback(const std_msgs::Bool::ConstPtr& msg){
		
		component->inB = msg->data;
		
		topic9Callback_wasCalled = true;
	}
	
	void publishtopic7Publisher(){
		std_msgs::Float64 tmpMsg;
		tmpMsg.data = component->outQ;
		topic7Publisher.publish(tmpMsg);
	}
	void publishtopic8Publisher(){
		std_msgs::Int32 tmpMsg;
		tmpMsg.data = component->outZ;
		topic8Publisher.publish(tmpMsg);
	}
	void publishtopic9Publisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->outB;
		topic9Publisher.publish(tmpMsg);
	}
	
	void tick(){
		topic7Callback_wasCalled = false;
		topic8Callback_wasCalled = false;
		topic9Callback_wasCalled = false;
		
		publishtopic7Publisher();
		publishtopic8Publisher();
		publishtopic9Publisher();
	}
	
};