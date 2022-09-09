/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "IAdapter_de_rwth_montisim_agent_master.h"
#include "de_rwth_montisim_agent_master.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

class RosAdapter_de_rwth_montisim_agent_master: public IAdapter_de_rwth_montisim_agent_master{
	
	de_rwth_montisim_agent_master* component;
	
	bool _sim_terminal2Callback_wasCalled;
	bool sim_state2Callback_wasCalled;
	
	ros::Subscriber _sim_terminal2Subscriber;
	ros::Subscriber sim_state2Subscriber;
	ros::Publisher _sim_reset2Publisher;
	ros::Publisher _sim_step2Publisher;
	
	public:
	RosAdapter_de_rwth_montisim_agent_master(){
		
	}
	
	void init(de_rwth_montisim_agent_master* comp){
		this->component = comp;
		_sim_terminal2Callback_wasCalled = false;
		sim_state2Callback_wasCalled = false;
		char* tmp = strdup("");
		int i = 0;
		ros::init(i, &tmp, "RosAdapter_de_rwth_montisim_agent_master_node");
		ros::NodeHandle node_handle = ros::NodeHandle();
		
		_sim_terminal2Subscriber = node_handle.subscribe("/sim/terminal2", 5, &RosAdapter_de_rwth_montisim_agent_master::_sim_terminal2Callback, this, ros::TransportHints().tcpNoDelay());
		sim_state2Subscriber = node_handle.subscribe("sim/state2", 5, &RosAdapter_de_rwth_montisim_agent_master::sim_state2Callback, this, ros::TransportHints().tcpNoDelay());
		
		_sim_reset2Publisher = node_handle.advertise<std_msgs::Bool>("/sim/reset2",5);
		_sim_step2Publisher = node_handle.advertise<std_msgs::Float32MultiArray>("/sim/step2",5);
		
		ros::spin();
	}
	
	bool hasReceivedNewData() {
		return true && _sim_terminal2Callback_wasCalled && sim_state2Callback_wasCalled;
	}
	
	void _sim_terminal2Callback(const std_msgs::Bool::ConstPtr& msg){
		
		component->terminated = msg->data;
		
		_sim_terminal2Callback_wasCalled = true;
	}
	void sim_state2Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
		
		int counter = 0;
		for(int i0 = 0; i0 < 25; i0++){
			if(0 <= counter && counter <= msg->data.size()-1){
				(component->state)(i0) = msg->data[counter];
			}
			else if(0 > counter){
				(component->state)(i0+msg->data.size()-1-0+1) = 0;
			}
			else{
				(component->state)(i0) = 0;
			}
			counter++;
		}
		
		
		sim_state2Callback_wasCalled = true;
	}
	
	void publish_sim_reset2Publisher(){
		std_msgs::Bool tmpMsg;
		tmpMsg.data = component->terminate;
		_sim_reset2Publisher.publish(tmpMsg);
	}
	void publish_sim_step2Publisher(){
		std_msgs::Float32MultiArray tmpMsg;
		tmpMsg.data.resize(3);
		int counter = 0;
		for(int i0 = 0; i0 < 3; i0++){
			tmpMsg.data[counter] = (component->action)(i0);
			counter++;
		}
		
		_sim_step2Publisher.publish(tmpMsg);
	}
	
	void tick(){
		_sim_terminal2Callback_wasCalled = false;
		sim_state2Callback_wasCalled = false;
		
		publish_sim_reset2Publisher();
		publish_sim_step2Publisher();
	}
	
};