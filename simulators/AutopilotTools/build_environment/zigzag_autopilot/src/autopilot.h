/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <vector>
#include "utils.h"
#include "buffer.h"
#include "json.h"


namespace autopilot {
    extern DynamicBuffer buffer; // Buffer used by all "writers" (json & binary) & Socket input buffer

    void set_port_json(int i, const char* data);
    void get_port_json(int i, JsonWriter &writer);
    void set_port_binary(int i, BinaryReader &br);
    void get_port_binary(int i, BinaryWriter &bw);

    // Methods
    void init();
    void execute(double delta_sec);

    extern const bool IS_SOCKET[];
    extern const bool IS_OUTPUT[];
    extern const int PORT_COUNT;
    extern const char* PROGRAM_INTERFACE;

    
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

    struct Autopilot {
        double true_velocity;
        vec2f64 true_position;
        double true_compass;
        std::vector<double> trajectory_x;
        int trajectory_x__size;
        std::vector<double> trajectory_y;
        int trajectory_y__size;
        double set_steering;
        double set_gas;
        double set_braking;

        double time;
        PID speed_pid;

        static constexpr double TARGET_VELOCITY = 30;
        static constexpr double MAX_STEERING = 30;
        static constexpr double MAX_VEHICLE_ACCEL = 7.460690;

        void init();
        void execute(double delta_sec);
    };
}
