/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_test_instanceArrayComp_basicPorts_1_.h"
#include "test_instanceArrayComp_basicPorts_1_.h"
#include <ros/ros.h>

class RosAdapter_test_instanceArrayComp_basicPorts_1_: public IAdapter_test_instanceArrayComp_basicPorts_1_{
	
	test_instanceArrayComp_basicPorts_1_* component;
	
	
	
	public:
	RosAdapter_test_instanceArrayComp_basicPorts_1_(){
		
	}
	
	void init(test_instanceArrayComp_basicPorts_1_* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_test_instanceArrayComp_basicPorts_1__node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true;
	}
	
	
	
	void tick(){
		
	}
	
};