/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <vector>
#include "utils.h"
#include <queue>
#include <string>

struct PID {
    double P;
    double I;
    double D;
    double previous_error = 0;
    double integral = 0;
    PID() {}
    PID(double P, double I, double D) : P(P), I(I), D(D) {}
    void reset(){
        previous_error = 0;
        integral = 0;
    }
    double compute(double dt, double current, double target) {
        /*
         * error := setpoint − measured_value integral := integral + error × dt
         * derivative := (error − previous_error) / dt output := Kp × error + Ki ×
         * integral + Kd × derivative previous_error := error
         */
        double error = target - current;
        integral += error * dt;
        double derivative = dt > 0 ? (error - previous_error) / dt : 0;
        previous_error = error;
        return P * error + I * integral + D * derivative;
    }
};

static constexpr auto BROADCAST_ADDR = "ff02::1";

struct SimplePacket1 {
    std::string addr; // Sender for incoming packets, target for outgoing ones.
    int payload;
};
struct SimplePacket2 {
    std::string addr;
    double payload;
};

struct Autopilot {
    // The Inputs
    double true_velocity;
    vec2f64 true_position;
    double true_compass;
    std::vector<double> trajectory_x;
    int trajectory_x__size;
    std::vector<double> trajectory_y;
    int trajectory_y__size;
    // The outputs
    double set_steering = 0;
    double set_gas = 0;
    double set_braking = 0;
    std::queue<SimplePacket1> netsock1_in;
    std::queue<SimplePacket1> netsock1_out;
    std::queue<SimplePacket2> netsock2_in;
    std::queue<SimplePacket2> netsock2_out;

    // Internal variables
    double time;
    PID speed_pid;
    double time_since_network;

    static constexpr double TARGET_VELOCITY = 30;
    static constexpr double MAX_STEERING = 30;
    static constexpr double MAX_VEHICLE_ACCEL = 7.460690;

    void init();

    // The main entry point for each cycle
    void execute(double delta_sec);
};