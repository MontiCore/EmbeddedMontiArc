#include "hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"

#include "emulator_manager.h"


using namespace std;




std::string HardwareEmulator::querry( const char *msg ) {
    return std::string();
}

bool HardwareEmulator::init( EmulatorManager &manager, const char *config ) {

    autopilot_name = "AutopilotAdapter";
    os_name = "";
    
    MessageParser parser( config );
    
    computer.debug.debug = false;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "autopilot" ) )
            autopilot_name = parser.get_string();
        else if ( parser.is_cmd( "os" ) )
            os_name = parser.get_string();
        else if ( parser.is_cmd( "debug" ) )
            setup_debug( parser );
    }
    
    if ( autopilot_name.size() == 0 ) {
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
    
    
    if ( os_name.compare( "windows" ) == 0 )
        computer.set_os( new OS::Windows() );
    else if ( os_name.compare( "linux" ) == 0 )
        computer.set_os( new OS::Linux() );
    else
        return false;
        
        
    if ( !computer.os->load_file( path.string().c_str() ) ) {
        error_msg = "Error loading autopilot";
        return false;
    }
    
    
    //Register port functions
    
    if ( !init_ports( input_ports, "get_input_count", "get_input_name", "get_input_type", "set_input_" ) )
        return false;
    if ( !init_ports( output_ports, "get_output_count", "get_output_name", "get_output_type", "get_output_" ) )
        return false;
        
    if ( !resolve( "init", init_address ) || !resolve( "execute", execute_address ) )
        return false;
        
    //TODO check if seperate call
    //computer.call( init_address );
    
    simulation_time = computer.computing_time;
    
    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    buffer_slot = section_stack.get_annotated( 1024, "Port Exchange buffer", Annotation::OBJECT );
    return true;
}

bool HardwareEmulator::resolve_autopilot_os( EmulatorManager &manager ) {

    path = manager.path + autopilot_name;
    
    
    if ( os_name.size() == 0 ) {
        bool found = false;
        //Check existing autopilots with this name
        for ( auto &e : manager.entries ) {
            auto &p = e.path();
            std::string i_file_name = p.filename().string();
            std::string i_extension = p.extension().string();
            if ( i_file_name.size() > i_extension.size() ) {
                auto raw_name = i_file_name.substr( 0, i_file_name.size() - i_extension.size() );
                if ( raw_name.compare( autopilot_name ) == 0 ) {
                    found = true;
                    if ( i_extension.compare( ".dll" ) == 0 )
                        os_name = "windows";
                    else if ( i_extension.compare( ".so" ) == 0 )
                        os_name = "linux";
                    break;
                }
            }
        }
        if ( !found ) {
            error_msg = "Could not find Autopilot: " + autopilot_name;
            return false;
        }
    }
    else {
        if ( os_name.compare( "windows" ) != 0 && os_name.compare( "linux" ) != 0 ) {
            error_msg = "Unsupported OS: " + os_name;
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
            computer.func_call->set_params( *( ulong * )&input.double_value );
            break;
        case VALUE_TYPE::INT:
            computer.func_call->set_params( ( ulong )input.int_value );
            break;
        case VALUE_TYPE::DOUBLE_ARRAY:
            if ( input.double_array.size * sizeof( double ) > buffer_slot.size ) {
                std::cerr << "JNIEmulator::call() with too big array" << std::endl;
                call_success = false;
                return;
            }
            computer.memory.write_memory( buffer_slot.start_address, input.double_array.size * sizeof( double ),
                                          ( uchar * )input.double_array.data.begin() );
            computer.func_call->set_params( buffer_slot.start_address, ( ulong )input.double_array.size );
            break;
    }
    
    call_success = computer.call( addr, input_ports[func_id].name.c_str() );
}

void HardwareEmulator::call_output( uint func_id ) {
    auto &addr = output_ports[func_id].function_address;
    auto &output = output_ports[func_id].buffer;
    
    call_success = computer.call( addr, output_ports[func_id].name.c_str() );
    if ( !call_success )
        return;
    ulong temp_v;
    switch ( output.type ) {
        case VALUE_TYPE::DOUBLE:
            temp_v = computer.func_call->get_return();
            output.double_value = *( double * ) & ( temp_v );
            break;
    }
}

void HardwareEmulator::call_void( uint64_t address, const char *name ) {
    call_success = computer.call( address, name );
}

bool HardwareEmulator::resolve( const std::string &name, uint64_t &target ) {
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type != Symbols::Symbol::EXPORT )
        return false;
    target = sym.addr;
    return true;
}


bool HardwareEmulator::init_ports( Array<Port> &ports, const char *get_count,
                                   const char *get_name, const char *get_type, const char *port_prefix ) {
    uint64_t get_name_addr, get_type_addr, get_count_addr;
    if ( !resolve( get_count, get_count_addr ) || !resolve( get_name, get_name_addr )
            || !resolve( get_type, get_type_addr ) )
        return false;
        
    computer.call( get_count_addr, get_count );
    uint port_count = ( uint )computer.func_call->get_return();
    ports.init( port_count );
    for ( auto i : urange( port_count ) ) {
        auto &port = ports[i];
        
        computer.func_call->set_params( i );
        computer.call( get_name_addr, get_name );
        port.name = ( char * )computer.memory.read_str( computer.func_call->get_return() );
        
        port_id.emplace( port.name, i );
        
        computer.func_call->set_params( i );
        computer.call( get_type_addr, get_type );
        char *type = ( char * )computer.memory.read_str( computer.func_call->get_return() );
        
        port.buffer.init( FunctionValue::get_type( type ) );
        port.updated = false;
        if ( !resolve( port_prefix + port.name, port.function_address ) )
            return false;
    }
    return true;
}

void HardwareEmulator::setup_debug( MessageParser &parser ) {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_code = false;
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
        else if ( next.compare( "code" ) == 0 )
            computer.debug.d_code = true;
    } while ( true );
}
