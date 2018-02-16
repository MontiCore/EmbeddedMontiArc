#pragma once
#include "IAdapter.h"
#include "tests_msg_basicStructComp.h"
#include <msgs/tests_structs_BasicStruct.h>
#include <ros/ros.h>
class RosAdapter_tests_msg_basicStructComp: public IAdapter{
	tests_msg_basicStructComp* component;
	ros::Subscriber topic1Subscriber;
	ros::Publisher topic2Publisher;
	
	public:
	RosAdapter_tests_msg_basicStructComp(){
		
	}
	
	void init(tests_msg_basicStructComp* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_msg_basicStructComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		topic1Subscriber = node_handle.subscribe("topic1" ,5,&RosAdapter_tests_msg_basicStructComp::topic1Callback, this, ros::TransportHints().tcpNoDelay());
		topic2Publisher = node_handle.advertise<msgs::tests_structs_BasicStruct>("topic2",5);
		ros::spin();
	}
	
	void publishtopic2Publisher(){
		msgs::tests_structs_BasicStruct tmpMsg;
		tmpMsg->fieldB1 = component->out1.fieldB1;
		tmpMsg->fieldQ1 = component->out1.fieldQ1;
		tmpMsg->fieldQ2 = component->out1.fieldQ2;
		tmpMsg->fieldZ1 = component->out1.fieldZ1;
		tmpMsg->fieldZ2 = component->out1.fieldZ2;
		topic2Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publishtopic2Publisher();
	}
	
	void topic1Callback(const msgs::tests_structs_BasicStruct::ConstPtr& msg){
		component->in1.fieldB1 = msg->fieldB1;
		component->in1.fieldQ1 = msg->fieldQ1;
		component->in1.fieldQ2 = msg->fieldQ2;
		component->in1.fieldZ1 = msg->fieldZ1;
		component->in1.fieldZ2 = msg->fieldZ2;
	}
	
};