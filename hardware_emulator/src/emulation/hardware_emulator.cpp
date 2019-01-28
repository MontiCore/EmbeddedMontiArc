#include "hardware_emulator.h"


using namespace std;




void HardwareEmulator::init( Computer &computer, const char *module_name ) {
    this->computer = &computer;
    this->module_name = module_name;
    jni_emu.init( computer );
    simulation_time = computer.computing_time;
    
    for ( uint input_id : Range( AUTOPILOT_INPUT_COUNT ) ) {
        auto &addr = inputs.function_address[input_id];
        auto &input = AutopilotFunction::autopilot_inputs[input_id];
        auto name = input.get_input_name();
        addr = computer.sys_calls.get_syscall( module_name, name );
        if ( addr == 0 )
            Log::err << "Couldn't find function " << name << "\n";
    }
    for ( uint output_id : Range( AUTOPILOT_OUTPUT_COUNT ) ) {
        auto &addr = outputs.function_address[output_id];
        auto &output = AutopilotFunction::autopilot_outputs[output_id];
        outputs.buffer[output_id].init( output.type );
        addr = computer.sys_calls.get_syscall( module_name, output.get_output_name() );
    }
    
    init_address = computer.sys_calls.get_syscall( module_name, AutopilotFunction::get_name( "init" ) );
    exec_address = computer.sys_calls.get_syscall( module_name, AutopilotFunction::get_name( "exec" ) );
    
    this->did_run = false;
}

void HardwareEmulator::set_inputs() {
    for ( auto input_id : Range( AUTOPILOT_INPUT_COUNT ) )
        call_input( input_id );
}

void HardwareEmulator::get_outputs() {
    for ( auto output_id : Range( AUTOPILOT_OUTPUT_COUNT ) )
        call_output( output_id );
}

void HardwareEmulator::exec() {
    if ( !computing() ) {
        if ( did_run ) {
            did_run = false;
            get_outputs();
        }
        
        set_inputs();
        call_void( exec_address );
        
        if ( computing() )
            did_run = true;
        else
            get_outputs();
    }
}


void HardwareEmulator::add_time( ulong delta ) {
    //If computer did not use all the available time, set its time 'starting point' to the current simulation time.
    if ( computer->computing_time < simulation_time )
        computer->computing_time = simulation_time;
    simulation_time += delta;
}

void HardwareEmulator::call_input( uint func_id ) {
    auto &addr = inputs.function_address[func_id];
    auto &input = inputs.buffer[func_id];
    
    switch ( input.type ) {
        case VALUE_TYPE::DOUBLE:
            call_success = jni_emu.call( addr, input.double_value );
            break;
        case VALUE_TYPE::INT:
            call_success = jni_emu.call( addr, ( ulong ) input.int_value );
            break;
        case VALUE_TYPE::DOUBLE_ARRAY:
            call_success = jni_emu.call( addr, input.double_array.data.begin(), input.double_array.size );
            break;
    }
}

void HardwareEmulator::call_output( uint func_id ) {
    auto &addr = outputs.function_address[func_id];
    auto &output = outputs.buffer[func_id];
    
    switch ( output.type ) {
        case VALUE_TYPE::DOUBLE:
            call_success = jni_emu.call( addr );
            output.double_value = jni_emu.return_double();
            break;
    }
}

void HardwareEmulator::call_void( uint64_t address ) {
    call_success = jni_emu.call( address );
}

