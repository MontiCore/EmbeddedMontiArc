/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "autopilot.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <iostream>
#include "err_out.h"

namespace autopilot {
using namespace std;

DynamicBuffer buffer;

Autopilot program_instance;

void set_port_json(int i, const char* data) {
    JsonTraverser reader{data};
    switch(i){
        case 0: { // true_velocity
            program_instance.true_velocity = reader.get_double();
        } break;
        case 1: { // true_position
            int i1 = 0;
            for (auto t : reader.stream_array()) {
                if (i == 0){
                    program_instance.true_position.x = reader.get_double();
                } else if (i == 1) {
                    program_instance.true_position.y = reader.get_double();
                } else {
                    // TODO error or warning?
                    break;
                }
                ++i;
            } if (i1 < 2) {} // TODO error?
        } break;
        case 2: { // true_compass
            program_instance.true_compass = reader.get_double();
        } break;
        case 3: { // trajectory_x
            bool first = true;
            int i = 0;
            for (auto t : reader.stream_array()) {
                if (first){
                    auto size = reader.get_long();
                    if (size >= 128) return;
                    program_instance.trajectory_x__size = size;
                    first = false;
                } else {
                    program_instance.trajectory_x[i++] = reader.get_double(); 
                }
            }
        } break;
        case 4: { // trajectory_y
            bool first = true;
            int i = 0;
            for (auto t : reader.stream_array()) {
                if (first){
                    auto size = reader.get_long();
                    if (size >= 128) return;
                    program_instance.trajectory_y__size = size;
                    first = false;
                } else {
                    program_instance.trajectory_y[i++] = reader.get_double(); 
                }
            }
        } break;

        default:
            cerr << "Invalid output port ID: '"<< i << "'" << endl;
            throw std::exception();
    }
}
void get_port_json(int i, JsonWriter &writer) {
    switch (i){
        case 5: { // set_gas
            writer.write_value(program_instance.set_gas);
        } break;
        case 6: { // set_steering
            writer.write_value(program_instance.set_steering);
        } break;
        case 7: { // set_braking
            writer.write_value(program_instance.set_braking);
        } break;

        default:
            cerr << "Invalid output port ID: '"<< i << "'" << endl;
            throw std::exception();
    }
}

void set_port_binary(int i, BinaryReader &reader) {
    switch (i){
        case 0: { // true_velocity
            program_instance.true_velocity = reader.read_f64();
        } break;
        case 1: { // true_position
            program_instance.true_position.x = reader.read_f64();
            program_instance.true_position.y = reader.read_f64();
        } break;
        case 2: { // true_compass
            program_instance.true_compass = reader.read_f64();
        } break;
        case 3: { // trajectory_x
            auto size1 = reader.read_u16();
            for (int i1=0; i1 < size1; ++i1) {
                auto &res1 = program_instance.trajectory_x[i1];
                res1 = reader.read_f64();
            }
        } break;
        case 4: { // trajectory_y
            auto size1 = reader.read_u16();
            for (int i1=0; i1 < size1; ++i1) {
                auto &res1 = program_instance.trajectory_y[i1];
                res1 = reader.read_f64();
            }
        } break;

        default:
            cerr << "Invalid output port ID: '"<< i << "'" << endl;
            throw std::exception();
    }
}
void get_port_binary(int i, BinaryWriter &writer) {
    switch (i){
        case 5: { // set_gas
            writer.write_f64(program_instance.set_gas);
        } break;
        case 6: { // set_steering
            writer.write_f64(program_instance.set_steering);
        } break;
        case 7: { // set_braking
            writer.write_f64(program_instance.set_braking);
        } break;

        default:
            cerr << "Invalid output port ID: '"<< i << "'" << endl;
            throw std::exception();
    }
}


void init() {
    program_instance.init();
}
void execute(double delta_sec) {
    program_instance.execute(delta_sec);
}

const int PORT_COUNT = 8;

const bool IS_SOCKET[] = {
    false, false, false, false, false, false, false, false
};

const bool IS_OUTPUT[] = {
    false, false, false, false, false, true, true, true
};

const char* PROGRAM_INTERFACE = R"(
{
"name": "cppautopilot",
"version": "2.0",
"ports": [{
    "name": "true_velocity", "direction": "INPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
},{
    "name": "true_position", "direction": "INPUT",
    "data_type": { "type": "basic", "base_type": "vec2" }
},{
    "name": "true_compass", "direction": "INPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
},{
    "name": "trajectory_x", "direction": "INPUT",
    "data_type": { "type": "vector", "base_type": { "type": "basic", "base_type": "Q" }, "size": 10 }
},{
    "name": "trajectory_y", "direction": "INPUT",
    "data_type": { "type": "vector", "base_type": { "type": "basic", "base_type": "Q" }, "size": 10 }
},{
    "name": "set_gas", "direction": "OUTPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
},{
    "name": "set_steering", "direction": "OUTPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
},{
    "name": "set_braking", "direction": "OUTPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
}]
}
)";


void Autopilot::init() {
    time = 0;
    true_velocity = 0;
    true_position.x = 0;
    true_position.y = 0;
    true_compass = 0;
    speed_pid = PID(1, 0, 0.2);
    trajectory_x.resize(10);
    trajectory_y.resize(10);

    print_cout("Hello World!");
    print_cerr("Cerr test");
}
// Simple Zig-Zag at TARGET_VELOCITY
void Autopilot::execute(double delta_sec) {

    // Go to constant 30 km/h
    double output = speed_pid.compute(delta_sec, true_velocity, TARGET_VELOCITY);
    output /= 3.6; // Convert to m/s related space
    set_gas = output / MAX_VEHICLE_ACCEL; // Convert to [0:1] actuator range

    // Turn in zig-zags
    set_steering = sin(time)*20;
    set_braking = 0.0;

    time += delta_sec;
}
}


