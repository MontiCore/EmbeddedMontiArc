/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_msg_multiNestedStructComp.h"
#include "tests_msg_multiNestedStructComp.h"
#include <ros/ros.h>
#include <struct_msgs/tests_structs_MultiNestedStruct.h>

class RosAdapter_tests_msg_multiNestedStructComp: public IAdapter_tests_msg_multiNestedStructComp{
	
	tests_msg_multiNestedStructComp* component;
	
	bool topic5Callback_wasCalled;
	
	ros::Subscriber topic5Subscriber;
	ros::Publisher topic6Publisher;
	
	public:
	RosAdapter_tests_msg_multiNestedStructComp(){
		
	}
	
	void init(tests_msg_multiNestedStructComp* comp){
		this->component = comp;
		topic5Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_msg_multiNestedStructComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		topic5Subscriber = node_handle.subscribe("topic5", 5, &RosAdapter_tests_msg_multiNestedStructComp::topic5Callback, this, ros::TransportHints().tcpNoDelay());
		
		topic6Publisher = node_handle.advertise<struct_msgs::tests_structs_MultiNestedStruct>("topic6",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && topic5Callback_wasCalled;
	}
	
	void topic5Callback(const struct_msgs::tests_structs_MultiNestedStruct::ConstPtr& msg){
		
		component->inMultiNested.fieldMultiNested1.fieldB = msg->fieldMultiNested1.fieldB;
		component->inMultiNested.fieldMultiNested1.fieldNested1.fieldB1 = msg->fieldMultiNested1.fieldNested1.fieldB1;
		component->inMultiNested.fieldMultiNested1.fieldNested1.fieldQ1 = msg->fieldMultiNested1.fieldNested1.fieldQ1;
		component->inMultiNested.fieldMultiNested1.fieldNested1.fieldQ2 = msg->fieldMultiNested1.fieldNested1.fieldQ2;
		component->inMultiNested.fieldMultiNested1.fieldNested1.fieldZ1 = msg->fieldMultiNested1.fieldNested1.fieldZ1;
		component->inMultiNested.fieldMultiNested1.fieldNested1.fieldZ2 = msg->fieldMultiNested1.fieldNested1.fieldZ2;
		component->inMultiNested.fieldMultiNested1.fieldNested2.fieldB1 = msg->fieldMultiNested1.fieldNested2.fieldB1;
		component->inMultiNested.fieldMultiNested1.fieldNested2.fieldQ1 = msg->fieldMultiNested1.fieldNested2.fieldQ1;
		component->inMultiNested.fieldMultiNested1.fieldNested2.fieldQ2 = msg->fieldMultiNested1.fieldNested2.fieldQ2;
		component->inMultiNested.fieldMultiNested1.fieldNested2.fieldZ1 = msg->fieldMultiNested1.fieldNested2.fieldZ1;
		component->inMultiNested.fieldMultiNested1.fieldNested2.fieldZ2 = msg->fieldMultiNested1.fieldNested2.fieldZ2;
		component->inMultiNested.fieldMultiNested1.fieldQ = msg->fieldMultiNested1.fieldQ;
		component->inMultiNested.fieldMultiNested1.fieldZ = msg->fieldMultiNested1.fieldZ;
		component->inMultiNested.fieldMultiNested2.fieldB = msg->fieldMultiNested2.fieldB;
		component->inMultiNested.fieldMultiNested2.fieldNested1.fieldB1 = msg->fieldMultiNested2.fieldNested1.fieldB1;
		component->inMultiNested.fieldMultiNested2.fieldNested1.fieldQ1 = msg->fieldMultiNested2.fieldNested1.fieldQ1;
		component->inMultiNested.fieldMultiNested2.fieldNested1.fieldQ2 = msg->fieldMultiNested2.fieldNested1.fieldQ2;
		component->inMultiNested.fieldMultiNested2.fieldNested1.fieldZ1 = msg->fieldMultiNested2.fieldNested1.fieldZ1;
		component->inMultiNested.fieldMultiNested2.fieldNested1.fieldZ2 = msg->fieldMultiNested2.fieldNested1.fieldZ2;
		component->inMultiNested.fieldMultiNested2.fieldNested2.fieldB1 = msg->fieldMultiNested2.fieldNested2.fieldB1;
		component->inMultiNested.fieldMultiNested2.fieldNested2.fieldQ1 = msg->fieldMultiNested2.fieldNested2.fieldQ1;
		component->inMultiNested.fieldMultiNested2.fieldNested2.fieldQ2 = msg->fieldMultiNested2.fieldNested2.fieldQ2;
		component->inMultiNested.fieldMultiNested2.fieldNested2.fieldZ1 = msg->fieldMultiNested2.fieldNested2.fieldZ1;
		component->inMultiNested.fieldMultiNested2.fieldNested2.fieldZ2 = msg->fieldMultiNested2.fieldNested2.fieldZ2;
		component->inMultiNested.fieldMultiNested2.fieldQ = msg->fieldMultiNested2.fieldQ;
		component->inMultiNested.fieldMultiNested2.fieldZ = msg->fieldMultiNested2.fieldZ;
		component->inMultiNested.fieldNested.fieldB1 = msg->fieldNested.fieldB1;
		component->inMultiNested.fieldNested.fieldQ1 = msg->fieldNested.fieldQ1;
		component->inMultiNested.fieldNested.fieldQ2 = msg->fieldNested.fieldQ2;
		component->inMultiNested.fieldNested.fieldZ1 = msg->fieldNested.fieldZ1;
		component->inMultiNested.fieldNested.fieldZ2 = msg->fieldNested.fieldZ2;
		component->inMultiNested.fieldQ = msg->fieldQ;
		
		topic5Callback_wasCalled = true;
	}
	
	void publishtopic6Publisher(){
		struct_msgs::tests_structs_MultiNestedStruct tmpMsg;
		tmpMsg.fieldMultiNested1.fieldB = component->outMultiNested.fieldMultiNested1.fieldB;
		tmpMsg.fieldMultiNested1.fieldNested1.fieldB1 = component->outMultiNested.fieldMultiNested1.fieldNested1.fieldB1;
		tmpMsg.fieldMultiNested1.fieldNested1.fieldQ1 = component->outMultiNested.fieldMultiNested1.fieldNested1.fieldQ1;
		tmpMsg.fieldMultiNested1.fieldNested1.fieldQ2 = component->outMultiNested.fieldMultiNested1.fieldNested1.fieldQ2;
		tmpMsg.fieldMultiNested1.fieldNested1.fieldZ1 = component->outMultiNested.fieldMultiNested1.fieldNested1.fieldZ1;
		tmpMsg.fieldMultiNested1.fieldNested1.fieldZ2 = component->outMultiNested.fieldMultiNested1.fieldNested1.fieldZ2;
		tmpMsg.fieldMultiNested1.fieldNested2.fieldB1 = component->outMultiNested.fieldMultiNested1.fieldNested2.fieldB1;
		tmpMsg.fieldMultiNested1.fieldNested2.fieldQ1 = component->outMultiNested.fieldMultiNested1.fieldNested2.fieldQ1;
		tmpMsg.fieldMultiNested1.fieldNested2.fieldQ2 = component->outMultiNested.fieldMultiNested1.fieldNested2.fieldQ2;
		tmpMsg.fieldMultiNested1.fieldNested2.fieldZ1 = component->outMultiNested.fieldMultiNested1.fieldNested2.fieldZ1;
		tmpMsg.fieldMultiNested1.fieldNested2.fieldZ2 = component->outMultiNested.fieldMultiNested1.fieldNested2.fieldZ2;
		tmpMsg.fieldMultiNested1.fieldQ = component->outMultiNested.fieldMultiNested1.fieldQ;
		tmpMsg.fieldMultiNested1.fieldZ = component->outMultiNested.fieldMultiNested1.fieldZ;
		tmpMsg.fieldMultiNested2.fieldB = component->outMultiNested.fieldMultiNested2.fieldB;
		tmpMsg.fieldMultiNested2.fieldNested1.fieldB1 = component->outMultiNested.fieldMultiNested2.fieldNested1.fieldB1;
		tmpMsg.fieldMultiNested2.fieldNested1.fieldQ1 = component->outMultiNested.fieldMultiNested2.fieldNested1.fieldQ1;
		tmpMsg.fieldMultiNested2.fieldNested1.fieldQ2 = component->outMultiNested.fieldMultiNested2.fieldNested1.fieldQ2;
		tmpMsg.fieldMultiNested2.fieldNested1.fieldZ1 = component->outMultiNested.fieldMultiNested2.fieldNested1.fieldZ1;
		tmpMsg.fieldMultiNested2.fieldNested1.fieldZ2 = component->outMultiNested.fieldMultiNested2.fieldNested1.fieldZ2;
		tmpMsg.fieldMultiNested2.fieldNested2.fieldB1 = component->outMultiNested.fieldMultiNested2.fieldNested2.fieldB1;
		tmpMsg.fieldMultiNested2.fieldNested2.fieldQ1 = component->outMultiNested.fieldMultiNested2.fieldNested2.fieldQ1;
		tmpMsg.fieldMultiNested2.fieldNested2.fieldQ2 = component->outMultiNested.fieldMultiNested2.fieldNested2.fieldQ2;
		tmpMsg.fieldMultiNested2.fieldNested2.fieldZ1 = component->outMultiNested.fieldMultiNested2.fieldNested2.fieldZ1;
		tmpMsg.fieldMultiNested2.fieldNested2.fieldZ2 = component->outMultiNested.fieldMultiNested2.fieldNested2.fieldZ2;
		tmpMsg.fieldMultiNested2.fieldQ = component->outMultiNested.fieldMultiNested2.fieldQ;
		tmpMsg.fieldMultiNested2.fieldZ = component->outMultiNested.fieldMultiNested2.fieldZ;
		tmpMsg.fieldNested.fieldB1 = component->outMultiNested.fieldNested.fieldB1;
		tmpMsg.fieldNested.fieldQ1 = component->outMultiNested.fieldNested.fieldQ1;
		tmpMsg.fieldNested.fieldQ2 = component->outMultiNested.fieldNested.fieldQ2;
		tmpMsg.fieldNested.fieldZ1 = component->outMultiNested.fieldNested.fieldZ1;
		tmpMsg.fieldNested.fieldZ2 = component->outMultiNested.fieldNested.fieldZ2;
		tmpMsg.fieldQ = component->outMultiNested.fieldQ;
		topic6Publisher.publish(tmpMsg);
	}
	
	void tick(){
		topic5Callback_wasCalled = false;
		
		publishtopic6Publisher();
	}
	
};