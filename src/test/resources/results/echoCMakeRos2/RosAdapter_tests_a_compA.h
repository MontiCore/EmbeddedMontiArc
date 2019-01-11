#pragma once
#include "IAdapter_tests_a_compA.h"
#include "tests_a_compA.h"
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
class RosAdapter_tests_a_compA: public IAdapter_tests_a_compA{
	tests_a_compA* component;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _clockSubscriber;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _echoPublisher;
	
	public:
	RosAdapter_tests_a_compA(){
		
	}
	
	void _clockCallback(const std_msgs::msg::Float64::SharedPtr msg){
		component->rosIn = msg->data;
	}
	
	void init(tests_a_compA* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 1;
		
		rclcpp::init(i, &tmp);
		auto node_handle = rclcpp::Node::make_shared("RosAdapter_tests_a_compA");
		
		_clockSubscriber = node_handle->create_subscription<std_msgs::msg::Float64>("/clock", std::bind(&RosAdapter_tests_a_compA::_clockCallback, this, std::placeholders::_1));
		_echoPublisher = node_handle->create_publisher<std_msgs::msg::Float64>("/echo");
		rclcpp::spin(node_handle);
	}
	
	void publish_echoPublisher(){
		std_msgs::msg::Float64 tmpMsg;
		tmpMsg.data = component->rosOut;
		_echoPublisher->publish(tmpMsg);
	}
	
	void tick(){
		publish_echoPublisher();
	}
	
};