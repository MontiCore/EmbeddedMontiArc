#pragma once
#include "IAdapter_ba_system_intersectionController.h"
#include "ba_system_intersectionController.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
class RosAdapter_ba_system_intersectionController: public IAdapter_ba_system_intersectionController{
	const int n = 2;
	const int x = 1;
	const int m = 10;
	const int m1 = 4;
	const int m2 = 10;
	ba_system_intersectionController* component;
	ros::Subscriber ba_system_intersectioncontroller_trajectoryin_1_Subscriber;
	ros::Subscriber ba_system_intersectioncontroller_trajectoryin_2_Subscriber;
	ros::Subscriber ba_system_intersectioncontroller_cutofftimeSubscriber;
	ros::Subscriber ba_system_intersectioncontroller_isactiveSubscriber;
	ros::Publisher _sim_comm_slowdown1Publisher;
	ros::Publisher _sim_comm_slowdown2Publisher;
	
	public:
	RosAdapter_ba_system_intersectionController(){
		
	}
	
	void ba_system_intersectionController_cutoffTimeCallback(const std_msgs::Float64::ConstPtr& msg){
		component->cutoffTime = msg->data;
	}
	
	void ba_system_intersectionController_isActiveCallback(const std_msgs::Bool::ConstPtr& msg){
		component->isActive = msg->data;
	}
	
	void ba_system_intersectionController_trajectoryIn_1_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		int counter = 0;
		for(int i0 = 0; i0 < 3; i0++){
			for(int i1 = 0; i1 < m; i1++){
				(component->trajectoryIn[0])(i0, i1) = msg->data[counter];
				counter++;
			}
		}
		
	}
	
	void ba_system_intersectionController_trajectoryIn_2_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
		int counter = 0;
		for(int i0 = 0; i0 < 3; i0++){
			for(int i1 = 0; i1 < m; i1++){
				(component->trajectoryIn[1])(i0, i1) = msg->data[counter];
				counter++;
			}
		}
		
	}
	
	void init(ba_system_intersectionController* comp){
		this->component = comp;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_ba_system_intersectionController_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		ba_system_intersectioncontroller_cutofftimeSubscriber = node_handle.subscribe("ba_system_intersectionController_cutoffTime" ,5,&RosAdapter_ba_system_intersectionController::ba_system_intersectionController_cutoffTimeCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_intersectioncontroller_isactiveSubscriber = node_handle.subscribe("ba_system_intersectionController_isActive" ,5,&RosAdapter_ba_system_intersectionController::ba_system_intersectionController_isActiveCallback, this, ros::TransportHints().tcpNoDelay());
		ba_system_intersectioncontroller_trajectoryin_1_Subscriber = node_handle.subscribe("ba_system_intersectionController_trajectoryIn_1_" ,5,&RosAdapter_ba_system_intersectionController::ba_system_intersectionController_trajectoryIn_1_Callback, this, ros::TransportHints().tcpNoDelay());
		ba_system_intersectioncontroller_trajectoryin_2_Subscriber = node_handle.subscribe("ba_system_intersectionController_trajectoryIn_2_" ,5,&RosAdapter_ba_system_intersectionController::ba_system_intersectionController_trajectoryIn_2_Callback, this, ros::TransportHints().tcpNoDelay());
		_sim_comm_slowdown1Publisher = node_handle.advertise<std_msgs::Bool>("/sim/comm/slowDown1",5);
		_sim_comm_slowdown2Publisher = node_handle.advertise<std_msgs::Bool>("/sim/comm/slowDown2",5);
		ros::spin();
	}
	
	void publish_sim_comm_slowdown1Publisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->stop[0];
		_sim_comm_slowdown1Publisher.publish(tmpMsg);
	}
	
	void publish_sim_comm_slowdown2Publisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->stop[1];
		_sim_comm_slowdown2Publisher.publish(tmpMsg);
	}
	
	void tick(){
		publish_sim_comm_slowdown1Publisher();
		publish_sim_comm_slowdown2Publisher();
	}
	
};