#pragma once
#include "utility.h"
#include "jni/jni_emulator.h"


struct HardwareEmulator {
    enum FunctionType {
        INPUT_DOUBLE,
        INPUT_INT,
        INPUT_ARRAY,
        OUTPUT
    };
    
    struct FunctionInfo {
        uint type;
        uint buffer_index;
        uint64_t address;
        bool new_input;
    };
    struct DoubleArray {
        Array<double> buffer;
        uint size;
    };
    
    const char *module_name;
    JNIEmulator jni_emu;
    Computer *computer;
    ulong simulation_time;
    
    Array<FunctionInfo> functions;
    
    Array<double> output_buffer;
    Array<double> current_output_buffer;
    
    Array<double> input_double_buffer;
    Array<int> input_int_buffer;
    Array<DoubleArray> input_array_buffer;
    uint input_double_pos;
    uint input_int_pos;
    uint input_array_pos;
    uint output_pos;
    
    bool call_success;
    
    bool computing() { //True when virtual computer still running in virtual time
        return simulation_time < computer->computing_time;
    }
    
    void init( Computer &computer, const char *module_name );
    
    void set_input_count( uint input_count );
    void set_output_count( uint output_count );
    void set_function_count( uint function_count );
    void reg_function( uint func, const char *name, uint type );
    
    void add_time( ulong delta ); //Check if computer stopped running => update outputs
    
    bool has_new_input( uint func_id ) {
        return functions[func_id].new_input;
    }
    void reset_new_input() {
        for ( auto &f : functions )
            f.new_input = false;
    }
    
    void set_input_double( uint func_id, double val );
    void set_input_int( uint func_id, int val );
    void set_input_array( uint func_id, double *vals, uint count );
    double get_output( uint func_id );
    
    void call( uint func_id );
};