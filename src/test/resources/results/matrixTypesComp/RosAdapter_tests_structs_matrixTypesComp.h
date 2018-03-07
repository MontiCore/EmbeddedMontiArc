#pragma once
#include "IAdapter_tests_structs_matrixTypesComp.h"
#include "tests_structs_matrixTypesComp.h"
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
class RosAdapter_tests_structs_matrixTypesComp: public IAdapter_tests_structs_matrixTypesComp{
	tests_structs_matrixTypesComp* component;
	ros::Subscriber _name1Subscriber;
	ros::Subscriber _name2Subscriber;
	ros::Publisher _name3Publisher;
	ros::Publisher _name4Publisher;
	
	public:
	RosAdapter_tests_structs_matrixTypesComp(){
		
	}
	
	void _name1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		(component->in1)(0, 0) = msg->data[0];
		(component->in1)(1, 0) = msg->data[1];
		(component->in1)(2, 0) = msg->data[2];
		
	}
	
	void _name2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		(component->in2)(0, 0) = msg->data[0] != 0;
		(component->in2)(0, 1) = msg->data[1] != 0;
		(component->in2)(0, 2) = msg->data[2] != 0;
		(component->in2)(0, 3) = msg->data[3] != 0;
		(component->in2)(1, 0) = msg->data[4] != 0;
		(component->in2)(1, 1) = msg->data[5] != 0;
		(component->in2)(1, 2) = msg->data[6] != 0;
		(component->in2)(1, 3) = msg->data[7] != 0;
		(component->in2)(2, 0) = msg->data[8] != 0;
		(component->in2)(2, 1) = msg->data[9] != 0;
		(component->in2)(2, 2) = msg->data[10] != 0;
		(component->in2)(2, 3) = msg->data[11] != 0;
		(component->in2)(3, 0) = msg->data[12] != 0;
		(component->in2)(3, 1) = msg->data[13] != 0;
		(component->in2)(3, 2) = msg->data[14] != 0;
		(component->in2)(3, 3) = msg->data[15] != 0;
		
	}
	
	void init(tests_structs_matrixTypesComp* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_structs_matrixTypesComp_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_name1Subscriber = node_handle.subscribe("/name1" ,5,&RosAdapter_tests_structs_matrixTypesComp::_name1Callback, this, ros::TransportHints().tcpNoDelay());
		_name2Subscriber = node_handle.subscribe("/name2" ,5,&RosAdapter_tests_structs_matrixTypesComp::_name2Callback, this, ros::TransportHints().tcpNoDelay());
		_name3Publisher = node_handle.advertise<std_msgs::ByteMultiArray>("/name3",5);
		_name4Publisher = node_handle.advertise<std_msgs::Int32MultiArray>("/name4",5);
		ros::spin();
	}
	
	void publish_name3Publisher(){
		std_msgs::ByteMultiArray tmpMsg;
		tmpMsg.data.resize(6);
		tmpMsg.data[0] = (component->out1)(0, 0) ? 1 : 0;
		tmpMsg.data[1] = (component->out1)(0, 1) ? 1 : 0;
		tmpMsg.data[2] = (component->out1)(0, 2) ? 1 : 0;
		tmpMsg.data[3] = (component->out1)(1, 0) ? 1 : 0;
		tmpMsg.data[4] = (component->out1)(1, 1) ? 1 : 0;
		tmpMsg.data[5] = (component->out1)(1, 2) ? 1 : 0;
		
		_name3Publisher.publish(tmpMsg);
	}
	
	void publish_name4Publisher(){
		std_msgs::Int32MultiArray tmpMsg;
		tmpMsg.data.resize(8);
		tmpMsg.data[0] = (component->out2)(0, 0);
		tmpMsg.data[1] = (component->out2)(0, 1);
		tmpMsg.data[2] = (component->out2)(0, 2);
		tmpMsg.data[3] = (component->out2)(0, 3);
		tmpMsg.data[4] = (component->out2)(1, 0);
		tmpMsg.data[5] = (component->out2)(1, 1);
		tmpMsg.data[6] = (component->out2)(1, 2);
		tmpMsg.data[7] = (component->out2)(1, 3);
		
		_name4Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_name3Publisher();
		publish_name4Publisher();
	}
	
};