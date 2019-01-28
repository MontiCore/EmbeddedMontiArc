#pragma once
#include "utility.h"
#include "jni/jni_emulator.h"
#include "autopilot/autopilot_functions.h"

struct HardwareEmulator {

    struct Inputs {
        FunctionValue buffer[AUTOPILOT_INPUT_COUNT];
        uint64_t function_address[AUTOPILOT_INPUT_COUNT];
    } inputs;
    
    struct Outputs {
        FunctionValue buffer[AUTOPILOT_OUTPUT_COUNT];
        uint64_t function_address[AUTOPILOT_OUTPUT_COUNT];
    } outputs;
    
    uint64_t init_address;
    uint64_t exec_address;
    
    Computer *computer;
    const char *module_name;
    JNIEmulator jni_emu;
    ulong simulation_time;
    
    bool call_success;
    bool did_run;
    
    bool computing() { //True when virtual computer still running in virtual time
        return simulation_time < computer->computing_time;
    }
    
    void init( Computer &computer, const char *module_name );
    
    void set_inputs();
    void get_outputs();
    void exec();
    
    void add_time( ulong delta ); //Check if computer stopped running => update outputs
    
    FunctionValue &get_input( uint input_id ) {
        return inputs.buffer[input_id];
    }
    
    FunctionValue &get_output( uint output_id ) {
        return outputs.buffer[output_id];
    }
    
    
    
    void call_input( uint func_id );
    void call_output( uint func_id );
    void call_void( uint64_t address );
};