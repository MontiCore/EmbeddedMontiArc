#pragma once
#include <ros/ros.h>
#include "tests_a_compA.h"
#include <rosgraph_msgs/Clock.h>
#include <automated_driving_msgs/StampedFloat64.h>
class tests_a_compA_RosWrapper{
	tests_a_compA component;
	ros::Subscriber _clockSubscriber;
	ros::Publisher _echoPublisher;

public:
	void _clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg){
		component.rosIn = msg->clock.toSec();
	}

	tests_a_compA_RosWrapper(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle){
		_clockSubscriber = node_handle.subscribe("/clock" ,5,&tests_a_compA_RosWrapper::_clockCallback, this, ros::TransportHints().tcpNoDelay());
		_echoPublisher = node_handle.advertise<automated_driving_msgs::StampedFloat64>("/echo",5);
	}

	void publish_echo(){
		automated_driving_msgs::StampedFloat64 tmpMsg;
		tmpMsg.data = component.rosOut;
		_echoPublisher.publish(tmpMsg);
	}

	void tick(){
		component.execute();
		publish_echo();
	}

};