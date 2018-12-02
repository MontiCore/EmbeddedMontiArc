#pragma once
#include "IAdapter_tests_a_compA.hpp"
#include "tests_a_compA.hpp"
#include <automated_driving_msgs/StampedFloat64.hpp>
#include <rclpp/rclpp.hpp>
#include <rosgraph_msgs/Clock.hpp>
class RosAdapter_tests_a_compA: public IAdapter_tests_a_compA{
	tests_a_compA* component;
	rclpp::Subscriber _clockSubscriber;
	rclpp::Publisher _echoPublisher;
	
	public:
	RosAdapter_tests_a_compA(){
		
	}
	
	void _clockCallback(const rosgraph_msgs::Clock::SharedPtr msg){
		component->rosIn = msg->clock.toSec();
	}
	
	void init(tests_a_compA* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		rclcpp::init(i, &tmp);
		_clockSubscriber = node_handle->create_subscription<rosgraph_msgs::Clock>("/clock", _clockCallback);
		_echoPublisher = node_handle->create_publisher<automated_driving_msgs::StampedFloat64>("/echo");
		rclcpp::spin(RosAdapter_tests_a_compA);
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