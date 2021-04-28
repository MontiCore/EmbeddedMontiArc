/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "autopilot.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <iostream>
using namespace std;

void Autopilot::init() {
    time = 0;
    true_velocity = 0;
    true_position.x = 0;
    true_position.y = 0;
    true_compass = 0;
    trajectory_x.resize(10);
    trajectory_y.resize(10);
    speed_pid = PID(1, 0, 0.2);
    time_since_network = (rand() % 100) *0.01;
}
// Simple Zig-Zag at TARGET_VELOCITY
void Autopilot::execute(double delta_sec) {
    //cout << "true_velocity: " << true_velocity << endl;
    if (time_since_network >= 1.0) {
        time_since_network = 0.0;
        // Broadcast a random int every second
        SimplePacket1 p1 = {BROADCAST_ADDR, rand()%100};
        netsock1_out.push(p1);
        std::cout << "BROADCASTING: " << p1.payload << " (addr: " << p1.addr << ")" <<std::endl;
    }

    while (!netsock1_in.empty()) {
        auto msg = netsock1_in.front();
        netsock1_in.pop();
        std::cout << "RECEIVED " << msg.payload << " from " << msg.addr <<" (sock1)" << std::endl;
        // Reply to sender on netsock2
        SimplePacket2 p2 = {msg.addr, msg.payload*0.05};
        netsock2_out.push(p2);
        std::cout << "REPLIED with " << p2.payload << std::endl;
    }
    while (!netsock2_in.empty()) {
        auto msg = netsock2_in.front();
        netsock2_in.pop();
        std::cout << "RECEIVED " << msg.payload << " from " << msg.addr <<" (sock2)" << std::endl;
    }

    // Go to constant 30 km/h
    double output = speed_pid.compute(delta_sec, true_velocity, TARGET_VELOCITY);
    output /= 3.6; // Convert to m/s related space
    set_gas = output / MAX_VEHICLE_ACCEL; // Convert to [0:1] actuator range

    // Turn in zig-zags
    set_steering = sin(time)*20;
    //cout << "set_steering: " << set_steering << endl;
    set_braking = 0;

    time += delta_sec;
    time_since_network += delta_sec;
}
