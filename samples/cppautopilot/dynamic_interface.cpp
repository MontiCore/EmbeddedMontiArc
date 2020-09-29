#include "dynamic_interface.h"
#include "autopilot.h"

const char *DI__port_names[] = { 
    "true_velocity",
    "true_position",
    "true_compass",
    "trajectory_x",
    "trajectory_y",
    "set_steering",
    "set_gas",
    "set_braking"
};


const char *DI__port_types[] = { 
    "{\"type\": \"basic\", \"base_type\": \"double\"}",
    "{\"type\": \"basic\", \"base_type\": \"vec2\"}",
    "{\"type\": \"basic\", \"base_type\": \"double\"}",
    "{\"type\": \"vector\", \"base_type\": \"double\", \"max_size\": \"128\"}",
    "{\"type\": \"vector\", \"base_type\": \"double\", \"max_size\": \"128\"}",
    "{\"type\": \"basic\", \"base_type\": \"double\"}",
    "{\"type\": \"basic\", \"base_type\": \"double\"}",
    "{\"type\": \"basic\", \"base_type\": \"double\"}"
};
bool DI__table__allows_multiple_inputs[] = {
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false,
};
bool DI__table__is_optional[] = {
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false,
};

// Dynamic Interface
int DI__get_port_count() {
    return 8;
}
bool DI__is_output(int i) {
    return i >= 5;
}
const char *DI__get_name(int i) {
    return DI__port_names[i];
}
const char *DI__get_type(int i) {
    return DI__port_types[i];
}
bool DI__allows_multiple_inputs(int i) {
    return DI__table__allows_multiple_inputs[i];
}
bool DI__is_optional(int i) {
    return DI__table__is_optional[i];
}

// Methods
void init() {
    autopilot.trajectory_x.resize(128);
    autopilot.trajectory_y.resize(128);
}
void execute() {

}




Autopilot autopilot;

void set__true_velocity(double v){
    autopilot.true_velocity = v;
}
void set__true_position(double v1, double v2){
    autopilot.true_position.x = v1;
    autopilot.true_position.y = v2;
}
void set__true_compass(double v){
    autopilot.true_compass = v;
}

void set_size__trajectory_x(int size) {
    if (size > 128) size = 128;
    autopilot.trajectory_x__size = size;
}
void set__trajectory_x(double v, int* i){
    if (*i >= 128) return;
    autopilot.trajectory_x[*i] = v;
}
void set_size__trajectory_y(int size) {
    if (size > 128) size = 128;
    autopilot.trajectory_y__size = size;
}
void set__trajectory_y(double v, int* i) {
    if (*i >= 128) return;
    autopilot.trajectory_y[*i] = v;
}
double get__set_steering(){
    return autopilot.set_steering;
}
double get__set_gas(){
    return autopilot.set_gas;
}
double get__set_braking(){
    return autopilot.set_braking;
}