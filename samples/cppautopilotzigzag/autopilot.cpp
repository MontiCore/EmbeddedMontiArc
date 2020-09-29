/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "autopilot.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>


void Autopilot::init() {
    time = 0;
    true_velocity = 0;
    true_position.x = 0;
    true_position.y = 0;
    true_compass = 0;
    speed_pid = PID(1, 0, 0.2);
}
// Simple Zig-Zag at TARGET_VELOCITY
void Autopilot::execute(double delta_sec) {

    // Go to constant 30 km/h
    double output = speed_pid.compute(delta_sec, true_velocity, TARGET_VELOCITY);
    output /= 3.6; // Convert to m/s related space
    set_gas = output / MAX_VEHICLE_ACCEL; // Convert to [0:1] actuator range

    // Turn in zig-zags
    set_steering = sin(time)*20;

    time += delta_sec;
}
