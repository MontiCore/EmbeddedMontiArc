/* (c) https://github.com/MontiCore/monticore */
//
// Created by nicola on 04.08.19.
//

#ifndef AGENT_DIRECT_DIRECTROSAGENT_H
#define AGENT_DIRECT_DIRECTROSAGENT_H


#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include "torcs_agent_torcsAgent/torcs_agent_torcsAgent.h"

class DirectRosAgent {
private:
    torcs_agent_torcsAgent component;

    ros::Publisher action_pub;
    ros::Subscriber state_sub;
public:
    DirectRosAgent();
    void init(int argc, char** argv);
    void state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};


#endif //AGENT_DIRECT_DIRECTROSAGENT_H
