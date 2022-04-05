/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_msg_basicStructComp.h"
#include "tests_msg_basicStructComp.h"
#include <ros/ros.h>
#include <struct_msgs/tests_structs_BasicStruct.h>

class RosAdapter_tests_msg_basicStructComp: public IAdapter_tests_msg_basicStructComp{
	
	tests_msg_basicStructComp* component;
	
	bool topic1Callback_wasCalled;
	
	ros::Subscriber topic1Subscriber;
	ros::Publisher topic2Publisher;
	
	public:
	RosAdapter_tests_msg_basicStructComp(){
		
	}
	
	void init(tests_msg_basicStructComp* comp){
		this->component = comp;
		topic1Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_msg_basicStructComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		topic1Subscriber = node_handle.subscribe("topic1", 5, &RosAdapter_tests_msg_basicStructComp::topic1Callback, this, ros::TransportHints().tcpNoDelay());
		
		topic2Publisher = node_handle.advertise<struct_msgs::tests_structs_BasicStruct>("topic2",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && topic1Callback_wasCalled;
	}
	
	void topic1Callback(const struct_msgs::tests_structs_BasicStruct::ConstPtr& msg){
		
		component->in1.fieldB1 = msg->fieldB1;
		component->in1.fieldQ1 = msg->fieldQ1;
		component->in1.fieldQ2 = msg->fieldQ2;
		component->in1.fieldZ1 = msg->fieldZ1;
		component->in1.fieldZ2 = msg->fieldZ2;
		
		topic1Callback_wasCalled = true;
	}
	
	void publishtopic2Publisher(){
		struct_msgs::tests_structs_BasicStruct tmpMsg;
		tmpMsg.fieldB1 = component->out1.fieldB1;
		tmpMsg.fieldQ1 = component->out1.fieldQ1;
		tmpMsg.fieldQ2 = component->out1.fieldQ2;
		tmpMsg.fieldZ1 = component->out1.fieldZ1;
		tmpMsg.fieldZ2 = component->out1.fieldZ2;
		topic2Publisher.publish(tmpMsg);
	}
	
	void tick(){
		topic1Callback_wasCalled = false;
		
		publishtopic2Publisher();
	}
	
};