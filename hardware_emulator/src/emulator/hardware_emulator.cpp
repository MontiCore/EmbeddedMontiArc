#include "hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"

#include "emulator_manager.h"


using namespace std;


std::string HardwareEmulator::query( const char *msg ) {
    MessageParser parser( msg );
    MessageBuilder builder;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "get_avg_runtime" ) )
            builder.add( "avg_runtime", std::to_string( avg_runtime.mean_avg() ) );
        else if ( parser.is_cmd( "is_computing" ) )
            builder.add( "computing", std::to_string( computing() ? 1 : 0 ) );
        else if ( parser.is_cmd( "get_computer_time" ) )
            builder.add( "computer_time", std::to_string( computer.time.micro_time ) );
        else
            parser.unknown();
    }
    return builder.res;
}

bool HardwareEmulator::init( EmulatorManager &manager, const char *config ) {
    //Setup default configuration values.
    simulation_time = 0;
    debug_time = false;
    test_real = false;
    export_data = false;
    autopilot_name = "AutopilotAdapter";
    os_name = "";
    computer.time.use_time = true;
    computer.debug.debug = false;
    
    //Read configuration
    Log::info << Log::tag << "HardwareEmulator::init() with config:\n" << config << "\n";
    MessageParser parser( config );
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "autopilot" ) )
            autopilot_name = parser.get_string();
        else if ( parser.is_cmd( "os" ) )
            os_name = parser.get_string();
        else if ( parser.is_cmd( "debug" ) )
            setup_debug( parser );
        else if ( parser.is_cmd( "cpu_frequency" ) ) {
            slong cpu_frequency;
            if ( parser.get_long( cpu_frequency ) )
                computer.time.set_cpu_frequency( cpu_frequency );
            else
                Log::err << "Could not read cpu_frequency config value\n";
        }
        else if ( parser.is_cmd( "memory_frequency" ) ) {
            slong memory_frequency;
            if ( parser.get_long( memory_frequency ) )
                computer.time.set_memory_frequency( memory_frequency );
            else
                Log::err << "Could not read cpu_frequency config value\n";
        }
        else if ( parser.is_cmd( "no_time" ) )
            computer.time.use_time = false;
        else if ( parser.is_cmd( "test_real" ) )
            test_real = true;
        else if ( parser.is_cmd( "export" ) )
            export_data = true;
        else if ( parser.cmd_starts_with( "cache_" ) )
            cache_settings.handle_config( parser );
        else
            parser.unknown();
    }
    
    //Validate configuration and set up the computer.
    
    
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
    
    
    if ( os_name.compare( "windows" ) == 0 ) {
        computer.set_os( new OS::Windows() );
        if ( Library::type != Library::OsType::WINDOWS )
            test_real = false;
    }
    else if ( os_name.compare( "linux" ) == 0 ) {
        computer.set_os( new OS::Linux() );
        if ( Library::type != Library::OsType::LINUX )
            test_real = false;
    }
    else
        return false;
        
        
    if ( !computer.os->load_file( path.string().c_str() ) ) {
        error_msg = "Error loading autopilot";
        return false;
    }
    
    if ( test_real ) {
        if ( !real_program.init( path.string().c_str() ) ) {
            error_msg = "Could not load real program";
            return false;
        }
    }
    
    
    //Register port functions
    
    if ( !init_ports( input_ports, "get_input_count", "get_input_name", "get_input_type", "set_input_" ) )
        return false;
    if ( !init_ports( output_ports, "get_output_count", "get_output_name", "get_output_type", "get_output_" ) )
        return false;
        
    if ( !resolve( "init", init_address ) || !resolve( "execute", execute_address ) )
        return false;
        
    if ( test_real )
        if ( !resolve_real( "init", real_init ) || !resolve_real( "execute", real_exec ) )
            return false;
            
    //TODO check if seperate call
    call_init();
    
    
    computer.time.reset();
    
    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    buffer_slot = section_stack.get_annotated( 1024, "Port Exchange buffer", Annotation::OBJECT );
    
    cache_settings.setup_computer( computer );
    
    Log::info << Log::tag << "Initiated emulator with autopilot: " << autopilot_name << ",\n"
              << "os: " << os_name << ",\n";
    if ( !computer.time.use_time )
        Log::info << "time disabled,\n";
    if ( test_real )
        Log::info << "real autopilot compared,\n";
    //debug_tick_count = 0;
    return true;
}

bool HardwareEmulator::resolve_autopilot_os( EmulatorManager &manager ) {

    path = manager.path.append( autopilot_name );
    
    
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


void HardwareEmulator::exec( ulong micro_delta ) {
    if ( !computing() || !computer.time.use_time ) {
        for ( auto &port : input_ports )
            call_input( port );
        call_execute();
        if ( debug_time )
            Log::info << Log::tag << "Execute time: " << computer.time.micro_time << "\n";
        execution_time = computer.time.micro_time;
    }
    
    if ( export_data )
        export_tick();
        
    //Upate time
    if ( !computer.time.use_time || computer.time.micro_time <= micro_delta ) {
        avg_runtime.add( computer.time.micro_time );
        
        //Upate outputs
        for ( auto &port : output_ports )
            call_output( port );
            
        computer.time.reset();
    }
    else
        computer.time.micro_time -= micro_delta;
        
    //++debug_tick_count;
    simulation_time += micro_delta;
}


HardwareEmulator::Port *HardwareEmulator::get_port( const char *port_name ) {
    auto res = port_map.find( port_name );
    if ( res == port_map.end() )
        return nullptr;
    return ( *res ).second;
}

void HardwareEmulator::call_input( Port &port ) {
    auto &addr = port.function_address;
    auto &input = port.buffer;
    
    switch ( input.type ) {
        case VALUE_TYPE::DOUBLE:
            computer.func_call->set_param1_double( input.double_value );
            if ( test_real ) {
                using DoubleInputFunc = void( * )( double );
                ( ( DoubleInputFunc )port.real_function )( input.double_value );
            }
            break;
        case VALUE_TYPE::INT:
            computer.func_call->set_param1_32( *( uint * )&input.int_value );
            if ( test_real ) {
                using IntInputFunc = void( * )( int );
                ( ( IntInputFunc )port.real_function )( *( uint * )&input.int_value );
            }
            break;
        case VALUE_TYPE::DOUBLE_ARRAY:
            if ( input.double_array.size * sizeof( double ) > buffer_slot.size ) {
                std::cerr << "JNIEmulator::call() with too big array" << std::endl;
                call_success = false;
                return;
            }
            computer.memory.write_memory( buffer_slot.start_address, input.double_array.size * sizeof( double ),
                                          ( uchar * )input.double_array.data.begin() );
            computer.func_call->set_params_64( buffer_slot.start_address, ( ulong )input.double_array.size );
            if ( test_real ) {
                using DoubleArrayInputFunc = void( * )( double *, int );
                ( ( DoubleArrayInputFunc )port.real_function )( input.double_array.data.begin(), input.double_array.size );
            }
            break;
        default:
            Log::err << Log::tag << "Add Port type support in HardwareEmulator::call_input().\n";
            break;
    }
    
    call_success = computer.call( addr, port.name.c_str() );
}

void compare_results( double real_value, double emulated_value, const char *port_name ) {
    if ( abs_t( real_value - emulated_value ) > 0.0001 )
        Log::err << Log::tag << "Desyncronisation detected on port " << port_name << ". "
                 "Real: " << real_value << " Emulated: " << emulated_value << "\n";
}

void HardwareEmulator::call_output( Port &port ) {
    auto &addr = port.function_address;
    auto &output = port.buffer;
    port.updated = true;
    
    call_success = computer.call( addr, port.name.c_str() );
    if ( !call_success )
        return;
    switch ( output.type ) {
        case VALUE_TYPE::DOUBLE:
            output.double_value = computer.func_call->get_return_double();
            if ( test_real ) {
                using DoubleOutputFunc = double( * )();
                compare_results( ( ( DoubleOutputFunc )port.real_function )(), output.double_value, port.name.c_str() );
            }
            break;
        default:
            Log::err << Log::tag << "Add Port type support in HardwareEmulator::call_output().\n";
            break;
    }
}

void HardwareEmulator::call_execute() {
    call_success = computer.call( execute_address, "execute" );
    if ( test_real ) {
        using ExecFunc = void( * )();
        ( ( ExecFunc )real_exec )();
    }
}

void HardwareEmulator::call_init() {
    call_success = computer.call( init_address, "init" );
    if ( test_real ) {
        using InitFunc = void( * )();
        ( ( InitFunc )real_init )();
    }
}

bool HardwareEmulator::resolve( const std::string &name, uint64_t &target ) {
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type != Symbols::Symbol::EXPORT ) {
        error_msg = "Could not resolve function: " + name;
        return false;
    }
    target = sym.addr;
    return true;
}

bool HardwareEmulator::resolve_real( const std::string &name, void *&target ) {
    target = real_program.get_function( name.c_str() );
    if ( target == nullptr ) {
        error_msg = "Could not resolve real function: " + name;
        return false;
    }
    return true;
}


bool HardwareEmulator::init_ports( Array<Port> &ports, const char *get_count,
                                   const char *get_name, const char *get_type, const char *port_prefix ) {
    uint64_t get_name_addr, get_type_addr, get_count_addr;
    if ( !resolve( get_count, get_count_addr ) || !resolve( get_name, get_name_addr )
            || !resolve( get_type, get_type_addr ) )
        return false;
        
    computer.call( get_count_addr, get_count );
    uint port_count = computer.func_call->get_return_32();
    ports.init( port_count );
    for ( auto i : urange( port_count ) ) {
        auto &port = ports[i];
        
        computer.func_call->set_param1_32( i );
        computer.call( get_name_addr, get_name );
        port.name = ( char * )computer.memory.read_str( computer.func_call->get_return_64() );
        
        port_map.emplace( port.name, &port );
        
        computer.func_call->set_param1_32( i );
        computer.call( get_type_addr, get_type );
        char *type = ( char * )computer.memory.read_str( computer.func_call->get_return_64() );
        
        port.buffer.init( FunctionValue::get_type( type ) );
        port.updated = false;
        auto func_name = port_prefix + port.name;
        if ( !resolve( func_name, port.function_address ) )
            return false;
        if ( test_real ) {
            if ( !resolve_real( func_name, port.real_function ) )
                return false;
        }
    }
    return true;
}

void HardwareEmulator::setup_debug( MessageParser &parser ) {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_code = false;
    computer.debug.d_regs = false;
    computer.debug.d_unsupported_syscalls = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    computer.debug.d_call = false;
    debug_time = false;
    
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
        else if ( next.compare( "unsupported_syscalls" ) == 0 )
            computer.debug.d_unsupported_syscalls = true;
        else if ( next.compare( "code" ) == 0 )
            computer.debug.d_code = true;
        else if ( next.compare( "call" ) == 0 )
            computer.debug.d_call = true;
        else if ( next.compare( "time" ) == 0 )
            debug_time = true;
        else
            Log::err << Log::tag << "Unknown debug flag\n";
    } while ( true );
}

void HardwareEmulator::export_tick() {
    if ( simulation_time > 60000000UL ) {
        if ( time_output.is_open() )
            time_output.close();
        if ( dist_output.is_open() )
            dist_output.close();
        return;
    }
    auto port_x = get_port( "x" );
    auto port_y = get_port( "y" );
    auto port_trajectory_x = get_port( "trajectory_x" );
    auto port_trajectory_y = get_port( "trajectory_y" );
    auto port_trajectory_length = get_port( "trajectory_length" );
    if ( port_x == nullptr || port_y == nullptr || port_trajectory_x == nullptr || port_trajectory_y == nullptr
            || port_trajectory_length == nullptr ) {
        Log::err << Log::tag << "Port resolve error for export_data()\n";
        return;
    }
    dvec2 pos;
    pos.x = port_x->buffer.double_value;
    pos.y = port_y->buffer.double_value;
    int &trajectory_length = port_trajectory_length->buffer.int_value;
    auto &trajectory_x = port_trajectory_x->buffer.double_array;
    auto &trajectory_y = port_trajectory_y->buffer.double_array;
    
    static constexpr auto inf = std::numeric_limits<double>::infinity();
    double final_dist = 0;
    if ( trajectory_length > 0 )
        final_dist = ( dvec2( trajectory_x.data[0], trajectory_y.data[0] ) - pos ).length();
        
    for ( uint i : urange( ( uint )trajectory_length - 1 ) ) {
        dvec2 pointA( trajectory_x.data[i], trajectory_y.data[i] );
        dvec2 pointB( trajectory_x.data[i + 1], trajectory_y.data[i + 1] );
        //Dist to line
        auto AP = pos - pointA;
        auto AB = pointB - pointA;
        auto size = AB.length();
        auto ABN = AB / size;
        auto proj = dot( ABN, AP );
        dvec2 perp( -ABN.y, ABN.x );
        auto perp_dist = dot( AP, perp );
        if ( proj > 0 && proj < size )
            minimize_abs( final_dist, perp_dist );
            
        double dist = ( pointB - pos ).length();
        if ( perp_dist < 0 )
            dist = -dist;
        minimize_abs( final_dist, dist );
    }
    
    double x_axis = ( simulation_time / 1000000.0 );
    if ( !dist_output.is_open() ) {
        dist_output.open( "autopilot_dist.txt", ios::trunc );
        dist_output << "x, y" << std::endl;
    }
    
    dist_output << x_axis << ", " << final_dist << std::endl;
    
    if ( !time_output.is_open() ) {
        time_output.open( "autopilot_time.txt", ios::trunc );
        time_output << "x, y" << std::endl;
    }
    time_output << x_axis << ", " << ( execution_time / 1000.0 ) << std::endl;
}

MemoryAccessInterface *setup_cache( Computer &computer, CacheSettings::Cache &cache ) {
    auto cache_layer = new FifoCache( computer.memory, cache.size );
    cache_layer->block_size = cache.block_size;
    cache_layer->set_ticks( computer.time, cache.read_ticks, cache.write_ticks );
    return cache_layer;
}

void CacheSettings::handle_config( MessageParser &parser ) {
    Cache *cache = nullptr;
    if ( parser.is_cmd( "cache_IL1" ) )
        cache = &IL1;
    else if ( parser.is_cmd( "cache_DL1" ) )
        cache = &DL1;
    else if ( parser.is_cmd( "cache_L2" ) )
        cache = &L2;
    else if ( parser.is_cmd( "cache_L3" ) )
        cache = &L3;
    else {
        Log::err << Log::tag << "Unknown cache level\n";
        return;
    }
    slong size;
    slong r_ticks;
    slong w_ticks;
    if ( parser.get_long( size ) ) {
        if ( size == 0 )
            *cache = Cache();
        else {
            if ( parser.get_long( r_ticks ) && parser.get_long( w_ticks ) )
                *cache = Cache( ( uint ) r_ticks, ( uint )w_ticks, ( uint )size );
            else
                Log::err << Log::tag << "Could not read tick parameters of cache config\n";
        }
    }
    else
        Log::err << Log::tag << "Could not read size parameter of cache config\n";
        
}

void CacheSettings::setup_computer( Computer &computer ) {
    if ( L3.used )
        computer.mem_model.add_common_interface( setup_cache( computer, L3 ) );
    if ( L2.used )
        computer.mem_model.add_common_interface( setup_cache( computer, L2 ) );
    if ( DL1.used )
        computer.mem_model.add_data_interface( setup_cache( computer, DL1 ) );
    if ( IL1.used )
        computer.mem_model.add_instruction_interface( setup_cache( computer, IL1 ) );
}
