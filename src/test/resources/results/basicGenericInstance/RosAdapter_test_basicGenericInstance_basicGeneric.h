/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_test_basicGenericInstance_basicGeneric.h"
#include "test_basicGenericInstance_basicGeneric.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class RosAdapter_test_basicGenericInstance_basicGeneric: public IAdapter_test_basicGenericInstance_basicGeneric{
	const int n = 3;
	
	test_basicGenericInstance_basicGeneric* component;
	ros::Subscriber _name1Subscriber;
	ros::Publisher _name1Publisher;
	
	public:
	RosAdapter_test_basicGenericInstance_basicGeneric(){
		
	}
	
	void init(test_basicGenericInstance_basicGeneric* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_test_basicGenericInstance_basicGeneric_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		_name1Subscriber = node_handle.subscribe("/name1", 5, &RosAdapter_test_basicGenericInstance_basicGeneric::_name1Callback, this, ros::TransportHints().tcpNoDelay());
		
		_name1Publisher = node_handle.advertise<std_msgs::Float64MultiArray>("/name1",5);
		
		ros::spin();
	}
	
	void _name1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < n; i0++){
			for(int i1 = 0; i1 < n; i1++){
				if(0 <= counter && counter <= n-1){
					(component->mat1)(i0, i1) = msg->data[counter];
				}
				else if(0 > counter){
					(component->mat1)(i0, i1+n-1-0+1) = 0;
				}
				else{
					(component->mat1)(i0, i1) = 0;
				}
				counter++;
			}
		}
		
		
	}
	
	void publish_name1Publisher(){
		std_msgs::Float64MultiArray tmpMsg;
		tmpMsg.data.resize(n * n);
		int counter = 0;
		for(int i0 = 0; i0 < n; i0++){
			for(int i1 = 0; i1 < n; i1++){
				tmpMsg.data[counter] = (component->matOut)(i0, i1);
				counter++;
			}
		}
		
		_name1Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_name1Publisher();
	}
	
};
