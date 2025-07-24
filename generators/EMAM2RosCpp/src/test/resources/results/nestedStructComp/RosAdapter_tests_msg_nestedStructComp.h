/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_msg_nestedStructComp.h"
#include "tests_msg_nestedStructComp.h"
#include <ros/ros.h>
#include <struct_msgs/tests_structs_NestedStruct.h>

class RosAdapter_tests_msg_nestedStructComp: public IAdapter_tests_msg_nestedStructComp{
	
	tests_msg_nestedStructComp* component;
	
	bool topic3Callback_wasCalled;
	
	ros::Subscriber topic3Subscriber;
	ros::Publisher topic4Publisher;
	
	public:
	RosAdapter_tests_msg_nestedStructComp(){
		
	}
	
	void init(tests_msg_nestedStructComp* comp){
		this->component = comp;
		topic3Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_msg_nestedStructComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		topic3Subscriber = node_handle.subscribe("topic3", 5, &RosAdapter_tests_msg_nestedStructComp::topic3Callback, this, ros::TransportHints().tcpNoDelay());
		
		topic4Publisher = node_handle.advertise<struct_msgs::tests_structs_NestedStruct>("topic4",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && topic3Callback_wasCalled;
	}
	
	void topic3Callback(const struct_msgs::tests_structs_NestedStruct::ConstPtr& msg){
		
		component->inNested.fieldB = msg->fieldB;
		component->inNested.fieldNested1.fieldB1 = msg->fieldNested1.fieldB1;
		component->inNested.fieldNested1.fieldQ1 = msg->fieldNested1.fieldQ1;
		component->inNested.fieldNested1.fieldQ2 = msg->fieldNested1.fieldQ2;
		component->inNested.fieldNested1.fieldZ1 = msg->fieldNested1.fieldZ1;
		component->inNested.fieldNested1.fieldZ2 = msg->fieldNested1.fieldZ2;
		component->inNested.fieldNested2.fieldB1 = msg->fieldNested2.fieldB1;
		component->inNested.fieldNested2.fieldQ1 = msg->fieldNested2.fieldQ1;
		component->inNested.fieldNested2.fieldQ2 = msg->fieldNested2.fieldQ2;
		component->inNested.fieldNested2.fieldZ1 = msg->fieldNested2.fieldZ1;
		component->inNested.fieldNested2.fieldZ2 = msg->fieldNested2.fieldZ2;
		component->inNested.fieldQ = msg->fieldQ;
		component->inNested.fieldZ = msg->fieldZ;
		
		topic3Callback_wasCalled = true;
	}
	
	void publishtopic4Publisher(){
		struct_msgs::tests_structs_NestedStruct tmpMsg;
		tmpMsg.fieldB = component->outNested.fieldB;
		tmpMsg.fieldNested1.fieldB1 = component->outNested.fieldNested1.fieldB1;
		tmpMsg.fieldNested1.fieldQ1 = component->outNested.fieldNested1.fieldQ1;
		tmpMsg.fieldNested1.fieldQ2 = component->outNested.fieldNested1.fieldQ2;
		tmpMsg.fieldNested1.fieldZ1 = component->outNested.fieldNested1.fieldZ1;
		tmpMsg.fieldNested1.fieldZ2 = component->outNested.fieldNested1.fieldZ2;
		tmpMsg.fieldNested2.fieldB1 = component->outNested.fieldNested2.fieldB1;
		tmpMsg.fieldNested2.fieldQ1 = component->outNested.fieldNested2.fieldQ1;
		tmpMsg.fieldNested2.fieldQ2 = component->outNested.fieldNested2.fieldQ2;
		tmpMsg.fieldNested2.fieldZ1 = component->outNested.fieldNested2.fieldZ1;
		tmpMsg.fieldNested2.fieldZ2 = component->outNested.fieldNested2.fieldZ2;
		tmpMsg.fieldQ = component->outNested.fieldQ;
		tmpMsg.fieldZ = component->outNested.fieldZ;
		topic4Publisher.publish(tmpMsg);
	}
	
	void tick(){
		topic3Callback_wasCalled = false;
		
		publishtopic4Publisher();
	}
	
};