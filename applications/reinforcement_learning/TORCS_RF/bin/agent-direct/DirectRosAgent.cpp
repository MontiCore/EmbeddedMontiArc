/* (c) https://github.com/MontiCore/monticore */
#include "DirectRosAgent.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>

DirectRosAgent::DirectRosAgent() = default;

void DirectRosAgent::init(int argc, char** argv) {
    component.init();
    ros::init(argc, argv, "TorcsDirectAgent");
    ros::NodeHandle n;

    action_pub = n.advertise<std_msgs::Float32MultiArray>("/torcs/step", 1);
    state_sub = n.subscribe("/torcs/state", 1, &DirectRosAgent::state_callback, this, ros::TransportHints().tcpNoDelay());

    ros::spin();
}

void DirectRosAgent::state_callback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    for (int i = 0; i < 29; i++) {
        component.state(i) = msg->data[i];
    }

    component.execute();

    std_msgs::Float32MultiArray tmpMsg;
    tmpMsg.data.resize(3);
    tmpMsg.data[0] = component.action[0];
    tmpMsg.data[1] = component.action[1];
    tmpMsg.data[2] = component.action[2];

    action_pub.publish(tmpMsg);
}
