#include "hardware_emulator.h"
#include "os_windows/windows_calls.h"

#include "emulator_manager.h"


using namespace std;




std::string HardwareEmulator::querry( const char *msg ) {
    return std::string();
}

bool HardwareEmulator::init( EmulatorManager &manager, const char *config ) {

    file_name = "AutopilotAdapter";
    os_name = "";
    
    MessageParser parser( config );
    
    computer.debug.debug = false;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "autopilot" ) )
            file_name = parser.get_string();
        else if ( parser.is_cmd( "os" ) )
            os_name = parser.get_string();
        else if ( parser.is_cmd( "debug" ) )
            setup_debug( parser );
    }
    
    if ( file_name.size() == 0 ) {
        error_msg = "Did not recieve Autopilot name";
        return false;
    }
    
    
    
    if ( !resolve_autopilot_os( manager ) )
        return false;
        
        
        
    computer.init();
    if ( !computer.loaded() ) {
        error_msg = "Could not initiate computer";
        return false;
    }
    
    
    
    
    
    os_windows.init( computer );
    WindowsCalls::add_windows_calls( computer.sys_calls, os_windows );
    if ( !os_windows.load_file( path.string().c_str() ) ) {
        error_msg = "Error loading autopilot";
        return false;
    }
    
    
    //Register port functions
    
    module_name = os_windows.dll.module_name;
    
    init_ports( input_ports, "get_input_count", "get_input_name", "get_input_type", "set_input_" );
    init_ports( output_ports, "get_output_count", "get_output_name", "get_output_type", "get_output_" );
    
    init_address = resolve( "init" );
    execute_address = resolve( "execute" );
    
    //TODO check if seperate call
    //computer.call( init_address );
    
    simulation_time = computer.computing_time;
    
    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    buffer_slot = section_stack.get_range( 1024 );
    section->annotations.add_annotation( buffer_slot, Annotation( "Port Exchange buffer", Annotation::SYMBOL ) );
    return true;
}

bool HardwareEmulator::resolve_autopilot_os( EmulatorManager &manager ) {
    //Check dll/so and type
    //=> validate / set OS type
    path = manager.path + file_name;
    auto ext = path.extension().string();
    if ( ext.size() == 0 ) {
        //No extension given: check if OS is specified
        if ( os_name.size() == 0 ) {
            bool found = false;
            //Check existing autopilots with this name
            for ( auto &e : manager.entries ) {
                auto &p = e.path();
                std::string file_name = p.filename().string();
                std::string extension = p.extension().string();
                if ( file_name.size() > extension.size() ) {
                    auto raw_name = file_name.substr( 0, file_name.size() - extension.size() );
                    if ( raw_name.compare( file_name ) == 0 ) {
                        found = true;
                        if ( ext.compare( ".dll" ) == 0 )
                            os_name = "windows";
                        else if ( ext.compare( ".so" ) == 0 )
                            os_name = "linux";
                        break;
                    }
                }
            }
            if ( !found ) {
                error_msg = "Could not find Autopilot: " + file_name;
                return false;
            }
        }
        else {
            if ( os_name.compare( "windows" ) == 0 )
                path += ".dll";
            else if ( os_name.compare( "windows" ) == 0 )
                path += ".so";
            else {
                error_msg = "Unsupported OS: " + os_name;
                return false;
            }
        }
    }
    else {
        bool compat_err = false;
        if ( ext.compare( ".dll" ) == 0 ) {
            if ( os_name.size() == 0 )
                os_name = "windows";
            else if ( os_name.compare( "windows" ) != 0 )
                compat_err = true;
        }
        else if ( ext.compare( ".so" ) == 0 ) {
            if ( os_name.size() == 0 )
                os_name = "linux";
            else if ( os_name.compare( "linux" ) != 0 )
                compat_err = true;
        }
        else {
            error_msg = "Unsupported autopilot extension: " + ext;
            return false;
        }
        if ( compat_err ) {
            error_msg = "Incompatible autopilot extension (" + ext + ") and OS (" + os_name + ")";
            return false;
        }
    }
    return true;
}


void HardwareEmulator::exec() {
    if ( !computing() ) {
        /*if ( did_run ) {
            did_run = false;
            get_outputs();
        }
        
        set_inputs();
        call_void( exec_address );
        
        if ( computing() )
            did_run = true;
        else
            get_outputs();*/
    }
}


void HardwareEmulator::add_time( ulong delta ) {
    //If computer did not use all the available time, set its time 'starting point' to the current simulation time.
    if ( computer.computing_time < simulation_time )
        computer.computing_time = simulation_time;
    simulation_time += delta;
}

int HardwareEmulator::get_port_id( const char *port_name ) {
    auto res = port_id.find( port_name );
    if ( res == port_id.end() )
        return -1;
    return ( *res ).second;
}

void HardwareEmulator::call_input( uint func_id ) {
    auto &addr = input_ports[func_id].function_address;
    auto &input = input_ports[func_id].buffer;
    
    switch ( input.type ) {
        case VALUE_TYPE::DOUBLE:
            computer.fast_call.set_params( *( ulong * )&input.double_value );
            break;
        case VALUE_TYPE::INT:
            computer.fast_call.set_params( ( ulong )input.int_value );
            break;
        case VALUE_TYPE::DOUBLE_ARRAY:
            if ( input.double_array.size * sizeof( double ) > buffer_slot.size ) {
                std::cerr << "JNIEmulator::call() with too big array" << std::endl;
                call_success = false;
                return;
            }
            computer.memory.write_memory( buffer_slot.start_address, input.double_array.size * sizeof( double ),
                                          ( uchar * )input.double_array.data.begin() );
            computer.fast_call.set_params( buffer_slot.start_address, ( ulong )input.double_array.size );
            break;
    }
    
    call_success = computer.call( addr );
}

void HardwareEmulator::call_output( uint func_id ) {
    auto &addr = output_ports[func_id].function_address;
    auto &output = output_ports[func_id].buffer;
    
    call_success = computer.call( addr );
    if ( !call_success )
        return;
    ulong temp_v;
    switch ( output.type ) {
        case VALUE_TYPE::DOUBLE:
            temp_v = computer.fast_call.get_return();
            output.double_value = *( double * ) & ( temp_v );
            break;
    }
}

void HardwareEmulator::call_void( uint64_t address ) {
    call_success = computer.call( address );
}

uint64_t HardwareEmulator::resolve( const std::string &name ) {
    return computer.sys_calls.get_syscall( module_name, name );
}

void HardwareEmulator::init_ports( Array<Port> &ports, const char *get_count,
                                   const char *get_name, const char *get_type, const char *port_prefix ) {
    computer.call( resolve( get_count ) );
    uint port_count = ( uint ) computer.fast_call.get_return();
    ports.init( port_count );
    auto get_name_addr = resolve( get_name );
    auto get_type_addr = resolve( get_type );
    for ( auto i : urange( port_count ) ) {
        auto &port = ports[i];
        
        computer.fast_call.set_params( i );
        computer.call( get_name_addr );
        port.name = ( char * )computer.memory.read_str( computer.fast_call.get_return() );
        
        port_id.emplace( port.name, i );
        
        computer.fast_call.set_params( i );
        computer.call( get_type_addr );
        char *type = ( char * )computer.memory.read_str( computer.fast_call.get_return() );
        
        port.buffer.init( FunctionValue::get_type( type ) );
        port.updated = false;
        port.function_address = resolve( port_prefix + port.name );
    }
}

void HardwareEmulator::setup_debug( MessageParser &parser ) {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    
    do {
        auto next = parser.get_string();
        if ( next.size() == 0 )
            return;
        if ( next.compare( "mem" ) == 0 )
            computer.debug.d_mem = true;
        else if ( next.compare( "regs" ) == 0 )
            computer.debug.d_regs = true;
        else if ( next.compare( "reg_update" ) == 0 )
            computer.debug.d_reg_update = true;
        else if ( next.compare( "syscalls" ) == 0 )
            computer.debug.d_syscalls = true;
    } while ( true );
}
