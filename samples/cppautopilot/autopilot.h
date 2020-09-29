#pragma once
#include <vector>
#include "utils.h"

struct Autopilot {
    double true_velocity;
    vec2<double> true_position;
    double true_compass;
    std::vector<double> trajectory_x;
    int trajectory_x__size;
    std::vector<double> trajectory_y;
    int trajectory_y__size;
    double set_steering;
    double set_gas;
    double set_braking;
};