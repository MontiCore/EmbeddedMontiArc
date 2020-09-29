/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <vector>
#include "utils.h"

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

struct Segment {
    int id; // Position of "start" in trajectory
    bool is_point = false; // Only "start" is used (+rel_pos)
    vec2f64 start;
    vec2f64 end;
    f64 length;
    vec2f64 dir; // Normalized vector for the direction of the segment
    vec2f64 normal; // Perpendicular "to the left" (in segment direction) of the segment
    vec2f64 rel_pos; // Relative position of the vehicle to the start of the segment (pointing to the vehicle position)
    f64 proj_pos; // Projected position along the segment (0 is start)
    f64 proj_pos_end; // Projected position along the segment reversed (0 is end, positive is towards start)
    f64 ortho_pos; // Orthogonal Position relative to "normal"
    f64 dist; // Distance to the segment

    void init_segment(int id, const vec2f64& start, const vec2f64& end, const vec2f64& vehicle_pos);
    void init_point(const vec2f64& position, const vec2f64& vehicle_pos);
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
    static constexpr double ROAD_WIDTH = 4;
    static constexpr double MAX_ROAD_SPEED = 50; // Km/h
    static constexpr double MAX_TURN_SIZE = 5;
    static constexpr double SAFE_TURN_FACTOR = 0.6;

    void init();
    void execute(double delta_sec);

    bool get_nearest_segment(const vec2f64& vehicle_pos, Segment &target);
    double get_max_turn_size(const Segment& current_segment, const Segment &next_segment);

    void follow_segment(const Segment& seg, double target_speed);
};