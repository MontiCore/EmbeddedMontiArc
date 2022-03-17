/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_structs_arrayHandlingComp.h"
#include "tests_structs_arrayHandlingComp.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

class RosAdapter_tests_structs_arrayHandlingComp: public IAdapter_tests_structs_arrayHandlingComp{
	
	tests_structs_arrayHandlingComp* component;
	
	bool _name1Callback_wasCalled;
	bool _name2Callback_wasCalled;
	bool _name3Callback_wasCalled;
	bool _name4Callback_wasCalled;
	
	ros::Subscriber _name1Subscriber;
	ros::Subscriber _name2Subscriber;
	ros::Subscriber _name3Subscriber;
	ros::Subscriber _name4Subscriber;
	ros::Publisher _name5Publisher;
	ros::Publisher _name6Publisher;
	
	public:
	RosAdapter_tests_structs_arrayHandlingComp(){
		
	}
	
	void init(tests_structs_arrayHandlingComp* comp){
		this->component = comp;
		_name1Callback_wasCalled = false;
		_name2Callback_wasCalled = false;
		_name3Callback_wasCalled = false;
		_name4Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_structs_arrayHandlingComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		_name1Subscriber = node_handle.subscribe("/name1", 5, &RosAdapter_tests_structs_arrayHandlingComp::_name1Callback, this, ros::TransportHints().tcpNoDelay());
		_name2Subscriber = node_handle.subscribe("/name2", 5, &RosAdapter_tests_structs_arrayHandlingComp::_name2Callback, this, ros::TransportHints().tcpNoDelay());
		_name3Subscriber = node_handle.subscribe("/name3", 5, &RosAdapter_tests_structs_arrayHandlingComp::_name3Callback, this, ros::TransportHints().tcpNoDelay());
		_name4Subscriber = node_handle.subscribe("/name4", 5, &RosAdapter_tests_structs_arrayHandlingComp::_name4Callback, this, ros::TransportHints().tcpNoDelay());
		
		_name5Publisher = node_handle.advertise<std_msgs::ByteMultiArray>("/name5",5);
		_name6Publisher = node_handle.advertise<std_msgs::Int32MultiArray>("/name6",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && _name1Callback_wasCalled && _name2Callback_wasCalled && _name3Callback_wasCalled && _name4Callback_wasCalled;
	}
	
	void _name1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < 3; i0++){
			for(int i1 = 0; i1 < 1; i1++){
				if(0 <= counter && counter <= msg->data.size()-1){
					(component->in1)(i0, i1-0) = msg->data[counter];
				}
				else if(0 > counter){
					(component->in1)(i0, i1+msg->data.size()-1-0+1) = 0;
				}
				else{
					(component->in1)(i0, i1) = 0;
				}
				counter++;
			}
		}
		
		
		_name1Callback_wasCalled = true;
	}
	void _name2Callback(const std_msgs::ByteMultiArray::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < 4; i0++){
			for(int i1 = 0; i1 < 4; i1++){
				if(0 <= counter && counter <= msg->data.size()-1){
					(component->in2)(i0, i1) = msg->data[counter] != 0;
				}
				else if(0 > counter){
					(component->in2)(i0, i1+msg->data.size()-1-0+1) = 0;
				}
				else{
					(component->in2)(i0, i1) = 0;
				}
				counter++;
			}
		}
		
		
		_name2Callback_wasCalled = true;
	}
	void _name3Callback(const nav_msgs::Path::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < 1; i0++){
			for(int i1 = 0; i1 < 10; i1++){
				if(0 <= counter && counter <= msg->data.size()-1){
					(component->in3)(i0, i1-0) = msg->poses[counter].pose.orientation.x;
				}
				else if(0 > counter){
					(component->in3)(i0, i1+msg->data.size()-1-0+1) = 0;
				}
				else{
					(component->in3)(i0, i1) = 0;
				}
				counter++;
			}
		}
		
		
		_name3Callback_wasCalled = true;
	}
	void _name4Callback(const nav_msgs::Path::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < 1; i0++){
			for(int i1 = 0; i1 < 7; i1++){
				if(2-1 <= counter && counter <= 6-1){
					(component->in4)(i0, i1-2-1) = msg->poses[counter].pose.orientation.x;
				}
				else if(2-1 > counter){
					(component->in4)(i0, i1+6-1-2-1+1) = 0;
				}
				else{
					(component->in4)(i0, i1) = 0;
				}
				counter++;
			}
		}
		
		
		_name4Callback_wasCalled = true;
	}
	
	void publish_name5Publisher(){
		std_msgs::ByteMultiArray tmpMsg;
		tmpMsg.data.resize(2 * 3);
		int counter = 0;
		for(int i0 = 0; i0 < 2; i0++){
			for(int i1 = 0; i1 < 3; i1++){
				tmpMsg.data[counter] = (component->out1)(i0, i1) ? 1 : 0;
				counter++;
			}
		}
		
		_name5Publisher.publish(tmpMsg);
	}
	void publish_name6Publisher(){
		std_msgs::Int32MultiArray tmpMsg;
		tmpMsg.data.resize(2 * 4);
		int counter = 0;
		for(int i0 = 0; i0 < 2; i0++){
			for(int i1 = 0; i1 < 4; i1++){
				tmpMsg.data[counter] = (component->out2)(i0, i1);
				counter++;
			}
		}
		
		_name6Publisher.publish(tmpMsg);
	}
	
	void tick(){
		_name1Callback_wasCalled = false;
		_name2Callback_wasCalled = false;
		_name3Callback_wasCalled = false;
		_name4Callback_wasCalled = false;
		
		publish_name5Publisher();
		publish_name6Publisher();
	}
	
};