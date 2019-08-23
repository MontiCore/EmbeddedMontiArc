/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_tests_a_compB.h"
#include "tests_a_compB.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
class RosAdapter_tests_a_compB: public IAdapter_tests_a_compB{
	tests_a_compB* component;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _clockSubscriber;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _echoPublisher;
	
	public:
	RosAdapter_tests_a_compB(){
		
	}

	void init(tests_a_compB* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 1;
		rclcpp::init(i, &tmp);
		auto node_handle = rclcpp::Node::make_shared("RosAdapter_tests_a_compB");
		_clockSubscriber = node_handle->create_subscription<std_msgs::msg::Float64>("/clock", std::bind(&RosAdapter_tests_a_compB::_clockCallback, this, std::placeholders::_1));
		_echoPublisher = node_handle->create_publisher<std_msgs::msg::Float64>("/echo");
		rclcpp::spin(node_handle);
	}

    void _clockCallback(const std_msgs::msg::Float64::SharedPtr msg){
        component->rosIn = msg->data;
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
