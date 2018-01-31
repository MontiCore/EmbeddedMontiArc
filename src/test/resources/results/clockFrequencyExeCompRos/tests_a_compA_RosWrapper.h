#pragma once
#include "tests_a_compA.h"
#include <automated_driving_msgs/StampedFloat64.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
class tests_a_compA_RosWrapper{
	tests_a_compA component;
	ros::Subscriber _clockSubscriber;
	ros::Publisher _echoPublisher;
	double lastTick;

public:
	void _clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg){
		component.rosIn = msg->clock.toSec();
		tickIfNeeded(msg);
	}

	tests_a_compA_RosWrapper(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle){
		_clockSubscriber = node_handle.subscribe("/clock" ,5,&tests_a_compA_RosWrapper::_clockCallback, this, ros::TransportHints().tcpNoDelay());
		_echoPublisher = node_handle.advertise<automated_driving_msgs::StampedFloat64>("/echo",5);
	}

	void publish0(){
		automated_driving_msgs::StampedFloat64 tmpMsg;
		tmpMsg.data = component.rosOut;
		_echoPublisher.publish(tmpMsg);
	}

	void tick(){
		component.execute();
		publish0();
	}

	void tickIfNeeded(const rosgraph_msgs::Clock::ConstPtr& clockMsg){
		double deltaT = 0.5;
		double sinceLast = clockMsg->clock.toSec() - this->lastTick;
		if(sinceLast >= deltaT){
			if(sinceLast / deltaT > 2) ROS_WARN("Missed Ticks!");
			this->lastTick = clockMsg->clock.toSec();
			tick();
		}

	}

};