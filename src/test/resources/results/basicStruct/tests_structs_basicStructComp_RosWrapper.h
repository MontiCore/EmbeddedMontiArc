#pragma once
#include "tests_structs_basicStructComp.h"
#include <package/type1.h>
#include <ros/ros.h>
class tests_structs_basicStructComp_RosWrapper{
	tests_structs_basicStructComp* component;
	ros::Subscriber name1Subscriber;

public:
	void name1Callback(const package::type1::ConstPtr& msg){
		component->posIn = msg->field1;
	}

	tests_structs_basicStructComp_RosWrapper(){
		
	}

	void init(tests_structs_basicStructComp* comp){
		this->component = comp;
		char* tmp = "";
		int i = 0;
		ros::init(i, &tmp, "tests_structs_basicStructComp_RosWrapper_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		name1Subscriber = node_handle.subscribe("name1" ,5,&tests_structs_basicStructComp_RosWrapper::name1Callback, this, ros::TransportHints().tcpNoDelay());
		ros::spin();
	}

	void tick();

};