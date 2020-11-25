#include "basic_interface.h"

using namespace std;



/*
    INTERFACE STRING
*/

string BASIC_INTERFACE = string(R"(
{
    "name":"basic_interface",
    "version":"1.0",
    "ports":[
        {"name":"true_velocity","type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"true_position","type":{"type":"basic","base_type":"vec2"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"true_compass","type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"trajectory_length","type":{"type":"basic","base_type":"N"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"trajectory_x","type":{"type":"vector","base_type":{"type":"basic","base_type":"Q"},"size":10},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"trajectory_y","type":{"type":"vector","base_type":{"type":"basic","base_type":"Q"},"size":10},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"steering","type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"gas","type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"braking","type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"set_steering","type":{"type":"basic","base_type":"Q"},"direction":"OUTPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"set_gas","type":{"type":"basic","base_type":"Q"},"direction":"OUTPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"set_braking","type":{"type":"basic","base_type":"Q"},"direction":"OUTPUT","allows_multiple_inputs":false,"optional":false}
    ]
}
)");