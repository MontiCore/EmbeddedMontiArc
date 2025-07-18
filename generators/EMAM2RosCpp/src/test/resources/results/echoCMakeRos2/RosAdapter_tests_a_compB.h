/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_a_compB.h"
#include "tests_a_compB.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class RosAdapter_tests_a_compB: public IAdapter_tests_a_compB{
	
	tests_a_compB* component;
	
	bool _clockCallback_wasCalled;
	
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _clockSubscriber;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _echoPublisher;
	
	public:
	RosAdapter_tests_a_compB(){
		
	}
	
	void init(tests_a_compB* comp){
		this->component = comp;
		_clockCallback_wasCalled = false;
		char* tmp = strdup("");
		int i = 1;
		rclcpp::init(i, &tmp);
		auto node_handle = rclcpp::Node::make_shared("RosAdapter_tests_a_compB");
		
		_clockSubscriber = node_handle->create_subscription<std_msgs::msg::Float64>("/clock", std::bind(&RosAdapter_tests_a_compB::_clockCallback, this, std::placeholders::_1));
		
		_echoPublisher = node_handle->create_publisher<std_msgs::msg::Float64>("/echo");
		
		rclcpp::spin(node_handle);
	}
	
	bool hasReceivedNewData() {
		return true && _clockCallback_wasCalled;
	}
	
	void _clockCallback(const std_msgs::msg::Float64::SharedPtr msg){
		component->rosIn = msg->data;
		_clockCallback_wasCalled = true;
	}
	
	void publish_echoPublisher(){
		std_msgs::msg::Float64 tmpMsg;
		tmpMsg.data = component->rosOut;
		_echoPublisher->publish(tmpMsg);
	}
	
	void tick(){
		_clockCallback_wasCalled = false;
		
		publish_echoPublisher();
	}
	
};