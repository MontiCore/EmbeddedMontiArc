#pragma once
#include "IAdapter.h"
#include "tests_a_compA.h"
#include <automated_driving_msgs/StampedFloat64.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
class tests_a_compA_RosWrapper: public IAdapter{
	tests_a_compA* component;
	ros::Subscriber _clockSubscriber;
	ros::Publisher _echoPublisher;

public:
	void _clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg){
		component->rosIn = msg->clock.toSec();
	}

	tests_a_compA_RosWrapper(){
		
	}

	void init(tests_a_compA* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "tests_a_compA_RosWrapper_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		_clockSubscriber = node_handle.subscribe("/clock" ,5,&tests_a_compA_RosWrapper::_clockCallback, this, ros::TransportHints().tcpNoDelay());
		_echoPublisher = node_handle.advertise<automated_driving_msgs::StampedFloat64>("/echo",5);
		ros::spin();
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