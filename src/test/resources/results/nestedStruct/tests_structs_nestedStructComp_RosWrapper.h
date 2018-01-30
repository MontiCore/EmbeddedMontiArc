#pragma once
#include "tests_structs_nestedStructComp.h"
#include <package/type2.h>
#include <ros/ros.h>
class tests_structs_nestedStructComp_RosWrapper{
	tests_structs_nestedStructComp component;
	ros::Subscriber name2Subscriber;

public:
	void name2Callback(const package::type2::ConstPtr& msg){
		component.posWithDtIn = msg->field2;
	}

	tests_structs_nestedStructComp_RosWrapper(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle){
		name2Subscriber = node_handle.subscribe("name2" ,5,&tests_structs_nestedStructComp_RosWrapper::name2Callback, this, ros::TransportHints().tcpNoDelay());
	}

	void tick(){
		component.execute();
	}

};