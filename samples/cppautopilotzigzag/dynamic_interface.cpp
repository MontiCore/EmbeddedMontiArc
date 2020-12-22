/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "dynamic_interface.h"
#include "autopilot.h"
#include "json.h"

Autopilot autopilot;
JsonWriter writer;

EXPORT const char* DI__get_interface() {
    return R"(
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
    "name": "set_steering", "direction": "OUTPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
},{
    "name": "set_gas", "direction": "OUTPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
},{
    "name": "set_braking", "direction": "OUTPUT",
    "data_type": { "type": "basic", "base_type": "Q" }
}]
})";
}
EXPORT void DI__set_port(int i, const char* data) {
    JsonTraverser traverser;
    traverser.init(data);
    switch(i){
        case 0: //true_velocity
            autopilot.true_velocity = traverser.get_double();
        break;
        case 1: //true_position
        {
            int i = 0;
            for (auto t : traverser.stream_array()){
                if (i == 0){
                    autopilot.true_position.x = traverser.get_double();
                } else if (i == 1) {
                    autopilot.true_position.y = traverser.get_double();
                } else {
                    // TODO error or warning?
                    break;
                }
                ++i;
            }
            if (i < 2){
                // TODO error or warning?
            }
        }
        break;
        case 2: //true_compass
            autopilot.true_compass = traverser.get_double();
        break;
        case 3: //trajectory_x 
        {
            bool first = true;
            int i = 0;
            for (auto t : traverser.stream_array()) {
                if (first){
                    auto size = traverser.get_long();
                    if (size >= 128) return;
                    autopilot.trajectory_x__size = size;
                    first = false;
                } else {
                    autopilot.trajectory_x[i++] = traverser.get_double(); 
                }
            }
        }
        break;
        case 4: //trajectory_y
        {
            bool first = true;
            int i = 0;
            for (auto t : traverser.stream_array()) {
                if (first){
                    auto size = traverser.get_long();
                    if (size >= 128) return;
                    autopilot.trajectory_y__size = size;
                    first = false;
                } else {
                    autopilot.trajectory_y[i++] = traverser.get_double(); 
                }
            }
        }
        break;
    }
}
EXPORT const char* DI__get_port(int i) {
    writer.init();
    switch (i){
        case 5: // set_steering
            writer.write_value(autopilot.set_steering);
        break;
        case 6: // set_gas
            writer.write_value(autopilot.set_gas);
        break;
        case 7: // set_braking
            writer.write_value(autopilot.set_braking);
        break;
    }
    return writer.get_string();
}


// Methods
EXPORT void DI__init() {
    writer.format = true;
    autopilot.init();
}
EXPORT void DI__execute(double delta_sec) {
    autopilot.execute(delta_sec);
}
