/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_a_compA.h"
#include "tests_a_compA.h"
#include <ros/ros.h>
#include <automated_driving_msgs/StampedFloat64.h>
#include <rosgraph_msgs/Clock.h>
class RosAdapter_tests_a_compA: public IAdapter_tests_a_compA{
	tests_a_compA* component;
	ros::Subscriber _clockSubscriber;
	ros::Publisher _echoPublisher;
	
	public:
	RosAdapter_tests_a_compA(){
		
	}
	
	void init(tests_a_compA* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_tests_a_compA_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_clockSubscriber = node_handle.subscribe("/clock", 5, &RosAdapter_tests_a_compA::_clockCallback, this, ros::TransportHints().tcpNoDelay());
		_echoPublisher = node_handle.advertise<automated_driving_msgs::StampedFloat64>("/echo",5);
		ros::spin();
	}

    void _clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg){
        component->rosIn = msg->clock.toSec();
    }

	void publish_echoPublisher(){
		automated_driving_msgs::StampedFloat64 tmpMsg;
		tmpMsg.data = component->rosOut;
		_echoPublisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_echoPublisher();
	}
	
};
