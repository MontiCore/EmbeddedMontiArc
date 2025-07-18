/* (c) https://github.com/MontiCore/monticore */
#include <iostream>
//#include "torcs_agent_torcsAgent/torcs_agent_torcsAgent.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "DirectRosAgent.h"

int main(int argc, char** argv) {
    DirectRosAgent agent;
    agent.init(argc, argv);
    return 0;
}
