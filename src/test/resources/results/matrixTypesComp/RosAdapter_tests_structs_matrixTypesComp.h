/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_structs_matrixTypesComp.h"
#include "tests_structs_matrixTypesComp.h"
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

class RosAdapter_tests_structs_matrixTypesComp: public IAdapter_tests_structs_matrixTypesComp{
	
	tests_structs_matrixTypesComp* component;
	
	bool _name1Callback_wasCalled;
	bool _name2Callback_wasCalled;
	
	ros::Subscriber _name1Subscriber;
	ros::Subscriber _name2Subscriber;
	ros::Publisher _name3Publisher;
	ros::Publisher _name4Publisher;
	
	public:
	RosAdapter_tests_structs_matrixTypesComp(){
		
	}
	
	void init(tests_structs_matrixTypesComp* comp){
		this->component = comp;
		_name1Callback_wasCalled = false;
		_name2Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_structs_matrixTypesComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		_name1Subscriber = node_handle.subscribe("/name1", 5, &RosAdapter_tests_structs_matrixTypesComp::_name1Callback, this, ros::TransportHints().tcpNoDelay());
		_name2Subscriber = node_handle.subscribe("/name2", 5, &RosAdapter_tests_structs_matrixTypesComp::_name2Callback, this, ros::TransportHints().tcpNoDelay());
		
		_name3Publisher = node_handle.advertise<std_msgs::ByteMultiArray>("/name3",5);
		_name4Publisher = node_handle.advertise<std_msgs::Int32MultiArray>("/name4",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && _name1Callback_wasCalled && _name2Callback_wasCalled;
	}
	
	void _name1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < 3; i0++){
			for(int i1 = 0; i1 < 1; i1++){
				if(0 <= counter && counter <= msg->data.size()-1){
					(component->in1)(i0, i1) = msg->data[counter];
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
	
	void publish_name3Publisher(){
		std_msgs::ByteMultiArray tmpMsg;
		tmpMsg.data.resize(2 * 3);
		int counter = 0;
		for(int i0 = 0; i0 < 2; i0++){
			for(int i1 = 0; i1 < 3; i1++){
				tmpMsg.data[counter] = (component->out1)(i0, i1) ? 1 : 0;
				counter++;
			}
		}
		
		_name3Publisher.publish(tmpMsg);
	}
	void publish_name4Publisher(){
		std_msgs::Int32MultiArray tmpMsg;
		tmpMsg.data.resize(2 * 4);
		int counter = 0;
		for(int i0 = 0; i0 < 2; i0++){
			for(int i1 = 0; i1 < 4; i1++){
				tmpMsg.data[counter] = (component->out2)(i0, i1);
				counter++;
			}
		}
		
		_name4Publisher.publish(tmpMsg);
	}
	
	void tick(){
		_name1Callback_wasCalled = false;
		_name2Callback_wasCalled = false;
		
		publish_name3Publisher();
		publish_name4Publisher();
	}
	
};