#include "hardware_emulator.h"


using namespace std;




void HardwareEmulator::init( Computer &computer, const char *module_name ) {
    this->computer = &computer;
    this->module_name = module_name;
    
    jni_emu.init( computer );
    simulation_time = computer.computing_time;
}

void HardwareEmulator::set_input_count( uint input_count ) {
    input_double_pos = 0;
    input_int_pos = 0;
    input_array_pos = 0;
    input_double_buffer.init( input_count );
    input_double_buffer.set_zero();
    input_int_buffer.init( input_count );
    input_int_buffer.set_zero();
    input_array_buffer.init( input_count );
    for ( auto &e : input_array_buffer )
        e.size = 0;
}

void HardwareEmulator::set_output_count( uint output_count ) {
    output_buffer.init( output_count );
    current_output_buffer.init( output_count );
    current_output_buffer.set_zero();
    output_pos = 0;
}

void HardwareEmulator::set_function_count( uint function_count ) {
    functions.init( function_count );
}

void HardwareEmulator::reg_function( uint func, const char *name, uint type ) {
    auto &f = functions[func];
    f.address = computer->sys_calls.get_syscall( module_name, name );
    f.type = type;
    switch ( type ) {
        case INPUT_DOUBLE:
            f.buffer_index = input_double_pos;
            input_double_pos++;
            break;
        case INPUT_INT:
            f.buffer_index = input_int_pos;
            input_int_pos++;
            break;
        case INPUT_ARRAY:
            f.buffer_index = input_array_pos;
            input_array_pos++;
            break;
        case OUTPUT:
            f.buffer_index = output_pos;
            output_pos++;
            break;
    }
}

void HardwareEmulator::add_time( ulong delta ) {
    //If computer did not use all the available time, set its time 'starting point' to the current simulation time.
    if ( computer->computing_time < simulation_time )
        computer->computing_time = simulation_time;
    bool was_comp = computing();
    simulation_time += delta;
    if ( was_comp && !computing() ) //Update output buffer
        for ( auto i : Range( output_buffer.size() ) )
            current_output_buffer[i] = output_buffer[i];
}

void HardwareEmulator::set_input_double( uint func_id, double val ) {
    auto &f = functions[func_id];
    input_double_buffer[f.buffer_index] = val;
    f.new_input = true;
}

void HardwareEmulator::set_input_int( uint func_id, int val ) {
    auto &f = functions[func_id];
    input_int_buffer[f.buffer_index] = val;
    f.new_input = true;
}

void HardwareEmulator::set_input_array( uint func_id, double *vals, uint count ) {
    auto &f = functions[func_id];
    auto &arr = input_array_buffer[f.buffer_index];
    if ( arr.buffer.size() < count )
        arr.buffer.init( count );
    for ( uint i : Range( count ) )
        arr.buffer[i] = vals[i];
    arr.size = count;
    f.new_input = true;
}

double HardwareEmulator::get_output( uint func_id ) {
    auto &f = functions[func_id];
    return output_buffer[f.buffer_index];
}

void HardwareEmulator::call( uint func_id ) {
    auto &f = functions[func_id];
    double *in;
    int *in2;
    double res;
    HardwareEmulator::DoubleArray *arr;
    switch ( f.type ) {
        case INPUT_DOUBLE:
            in = &( input_double_buffer[f.buffer_index] );
            call_success = jni_emu.call( f.address, *in );
            break;
        case INPUT_INT:
            in2 = &( input_int_buffer[f.buffer_index] );
            call_success = jni_emu.call( f.address, ( ulong ) * in2 );
            break;
        case INPUT_ARRAY:
            arr = &( input_array_buffer[f.buffer_index] );
            call_success = jni_emu.call( f.address, arr->buffer.begin(), arr->size );
            break;
        case OUTPUT:
            call_success = jni_emu.call( f.address );
            res = jni_emu.return_double();
            output_buffer[f.buffer_index] = res;
            if ( !computing() )
                current_output_buffer[f.buffer_index] = res;
            break;
        default:
            call_success = jni_emu.call( f.address );
    }
}
