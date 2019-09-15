/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "tests.h"
#include "dll_interface.h"
#include "emulator/emulator_manager.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"
#include "config.h"
#include "os_linux/elf.h"
#include "utility.h"


bool test_filesystem(){
    auto cp = FS::current_path();
    Log::debug << "FS::current_path(): " << cp << "\n";
    Log::debug << "FS::canonical(FS::current_path()): " << FS::canonical(cp) << "\n";
    auto files = FS::directory_files(cp);
    Log::debug << "FS::directory_files(FS::current_path()):\n";
    for (const auto &file : files){
        Log::debug << "\t" << file.path << ": " << file.name << " ext: " << file.extension << "\n";
    }
    return true;
}

/*
    Tests the emulation of the functions from
    samples/simple/
    depending of the os type
*/
bool test_simple_sample( OS::OS *os, bool windows ) {
    Computer computer;
    computer.init();
    if ( !computer.loaded() )
        return false;
        
        
    computer.set_os( os );
    
    
    ADD_DLL::Interface interf;
    if ( !interf.init( computer, windows ) )
        return false;
        
    //computer.debug.debug = true;
    //computer.debug.d_code = true;
    //computer.debug.d_mem = true;
    ////computer.debug.d_regs = false;
    //computer.debug.d_reg_update = true;
    //computer.debug.d_syscalls = true;
    //computer.debug.d_call = true;
    Log::debug << "add(2,3):\n";
    auto res = interf.add( 2, 3 );
    Log::debug << "Result=" << res << "\n";
    
    return interf.call_success;
}


#include "../../samples/funccalling/algorithm.h"

/*
        Tests the emulation of the functions from
        samples/funccalling/
        depending of the os type

        The test compares and validates the results by computing the content of the functions
        using the algorithms from samples/funccalling/algorithm.h
*/
bool test_funccalling_sample( OS::OS *os ) {
    Computer computer;
    computer.debug.debug = false;
    computer.debug.d_code = false;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    
    computer.init();
    if ( !computer.loaded() )
        return false;
        
    computer.set_os( os );
    
    if ( !computer.os->load_file( "sample_functioncalling" ) )
        return false;
        
    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    auto buffer_slot = section_stack.get_annotated( 1024, "Port Exchange buffer", Annotation::OBJECT );
    
#define TEST_FUNCCALL(FUNC_NAME, PARAM_TYPE, COMBINE_TYPE, ...) \
    auto func_##FUNC_NAME = computer.symbols.get_symbol( #FUNC_NAME ); \
    if (func_##FUNC_NAME .type == Symbols::Symbol::NONE ) { \
        Log::err << Log::tag << "Could not find function symbol for " #FUNC_NAME "\n"; \
        return false; \
    } \
    computer.func_call->set_params_##PARAM_TYPE (__VA_ARGS__); \
    if ( !computer.call(func_##FUNC_NAME .addr, #FUNC_NAME) ) { \
        Log::err << Log::tag << "Error executing " #FUNC_NAME "\n"; \
        return false; \
    } \
    auto res_##FUNC_NAME = computer.func_call->get_return_##PARAM_TYPE(); \
    auto verify_##FUNC_NAME = combine< COMBINE_TYPE >(__VA_ARGS__); \
    if ( res_##FUNC_NAME != verify_##FUNC_NAME) { \
        Log::err << Log::tag << "Error verifying " #FUNC_NAME ":\nResult: " << res_##FUNC_NAME << " != Verify: " << \
                 verify_##FUNC_NAME << "\n"; \
        return false; \
    }
    
    TEST_FUNCCALL( int_one, 32, int32_t, 24 );
    TEST_FUNCCALL( int_two, 32, int32_t, 24, 13 );
    TEST_FUNCCALL( int_three, 32, int32_t, 24, 13, 11 );
    TEST_FUNCCALL( int_four, 32, int32_t, 24, 13, 11, 31 );
    /* computer.debug.debug = true;
    computer.debug.d_code = true;
    computer.debug.d_mem = true;
    computer.debug.d_reg_update = true;
    computer.debug.d_syscalls = true; */
    
    TEST_FUNCCALL( long_one, 64, int64_t, 24 );
    TEST_FUNCCALL( long_two, 64, int64_t, 24, 13 );
    TEST_FUNCCALL( long_three, 64, int64_t, 24, 13, 11 );
    TEST_FUNCCALL( long_four, 64, int64_t, 24, 13, 11, 31 );
    
    
    TEST_FUNCCALL( float_one, float, float, 24 );
    
    
    TEST_FUNCCALL( float_two, float, float, 24, 13 );
    TEST_FUNCCALL( float_three, float, float, 24, 13, 11 );
    TEST_FUNCCALL( float_four, float, float, 24, 13, 11, 31 );
    
    TEST_FUNCCALL( double_one, double, double, 24 );
    TEST_FUNCCALL( double_two, double, double, 24, 13 );
    TEST_FUNCCALL( double_three, double, double, 24, 13, 11 );
    TEST_FUNCCALL( double_four, double, double, 24, 13, 11, 31 );
    
#undef TEST_FUNCCALL
    
#define TEST_ARRAY_FUNCCALL(FUNC_NAME, PARAM_NAME, PARAM_TYPE) \
    auto func_##FUNC_NAME = computer.symbols.get_symbol( #FUNC_NAME ); \
    if (func_##FUNC_NAME .type == Symbols::Symbol::NONE ) { \
        Log::err << Log::tag << "Could not find function symbol for " #FUNC_NAME "\n"; \
        return false; \
    } \
    PARAM_TYPE local_array_##FUNC_NAME [5] = { 17, 20, 12, 7, 8 }; \
    computer.memory.write_memory(buffer_slot.start_address, 5 * sizeof( PARAM_TYPE ), (uchar *)local_array_##FUNC_NAME ); \
    computer.func_call->set_param1_64(buffer_slot.start_address); \
    computer.func_call->set_param2_32(5); \
    if ( !computer.call(func_##FUNC_NAME .addr, #FUNC_NAME) ) { \
        Log::err << Log::tag << "Error executing " #FUNC_NAME "\n"; \
        return false; \
    } \
    auto res_##FUNC_NAME = computer.func_call->get_return_##PARAM_NAME(); \
    auto verify_##FUNC_NAME = combine_array< PARAM_TYPE >(local_array_##FUNC_NAME , 5); \
    if ( res_##FUNC_NAME != verify_##FUNC_NAME) { \
        Log::err << Log::tag << "Error verifying " #FUNC_NAME ":\nResult: " << res_##FUNC_NAME << " != Verify: " << \
                 verify_##FUNC_NAME << "\n"; \
        return false; \
    }
    TEST_ARRAY_FUNCCALL( double_array, double, double );
    TEST_ARRAY_FUNCCALL( float_array, float, float );
    TEST_ARRAY_FUNCCALL( int_array, 32, int32_t );
    TEST_ARRAY_FUNCCALL( long_array, 64, int64_t );
    TEST_ARRAY_FUNCCALL( char_array, char, char );
    
#undef TEST_ARRAY_FUNCCALL
    return true;
}




/*
    Tests the emulation of the functions from
    samples/syscall_dll/sample_syscall.dll
    samples/syscall_so/sample_syscall.so
    depending of the os type
*/
bool test_syscall_sample( OS::OS *os ) {
    Computer computer;
    computer.debug.debug = false;
    computer.debug.d_code = false;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    
    computer.init();
    if ( !computer.loaded() )
        return false;
        
    computer.set_os( os );
    
    
    
    LOADED_DLL::Interface interf;
    if ( !interf.init( computer ) )
        return false;
        
    /*computer.debug.debug = true;
    computer.debug.d_code = true;
    computer.debug.d_mem = true;
    computer.debug.d_reg_update = true;
    computer.debug.d_syscalls = true;*/
    Log::debug << "test_method()\n";
    interf.test_method();
    
    return interf.call_success;
}


bool test_simple_dll() {
    return test_simple_sample( new OS::Windows(), true );
}

bool test_funccalling_dll() {
    return test_funccalling_sample( new OS::Windows() );
}

bool test_syscall_dll() {
    return test_syscall_sample( new OS::Windows() );
}

bool test_hardware_manager_querries() {
    EmulatorManager manager;
    if ( !manager.init( "", "" ) ) {
        Log::err << "Could not initiate EmulatorManager\n";
        return false;
    }
    
    MessageBuilder builder;
    builder.add( "get_available_autopilots" );
    builder.add( "get_available_threads" );
    auto query_res = manager.query( builder.res.c_str() );
    MessageParser parser( query_res.c_str() );
    bool has_threads = false;
    bool has_autopilots = false;
    
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "available_threads" ) ) {
            slong thread_count;
            if ( !parser.get_long( thread_count ) )
                Log::err << "Did not recieve integer from 'available_threads' query to EmulatorManager\n";
            else {
                Log::debug << "Available threads: " << thread_count << '\n';
                has_threads = true;
            }
        }
        else if ( parser.is_cmd( "available_autopilots" ) ) {
            auto autopilots = parser.get_string();
            if ( autopilots.size() == 0 ) {
                //Should at least contain the dlls used for earlier tests
                Log::err << "Did not recieve string from 'get_available_autopilots' query to EmulatorManager\n";
            }
            else {
                Log::debug << "Available autopilots:\n" << autopilots << '\n';
                has_autopilots = true;
            }
        }
    }
    
    if ( !has_threads )
        Log::err << "Did not recieve response from 'get_available_threads' query to EmulatorManager\n";
    if ( !has_autopilots )
        Log::err << "Did not recieve response from 'get_available_autopilots' query to EmulatorManager\n";
        
    if ( !has_threads || !has_autopilots )
        return false;
        
    return true;
}

bool test_input_double( HardwareEmulator &emulator, const char *name, double value ) {
    auto port_ptr = emulator.get_port( name );
    if ( port_ptr == nullptr ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = *port_ptr;
    if ( port.buffer.type != VALUE_TYPE::DOUBLE ) {
        Log::err << "Type of port: " << name << " not 'double'\n";
        return false;
    }
    port.buffer.double_value = value;
    
    emulator.call_input( port );
    
    if ( !emulator.call_success ) {
        Log::err << "Could not set input: " << name << " ( emulator.call_input() )\n";
        return false;
    }
    return true;
}

bool test_input_int( HardwareEmulator &emulator, const char *name, int value ) {
    auto port_ptr = emulator.get_port( name );
    if ( port_ptr == nullptr ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = *port_ptr;
    if ( port.buffer.type != VALUE_TYPE::INT ) {
        Log::err << "Type of port: " << name << " not 'int'\n";
        return false;
    }
    port.buffer.int_value = value;
    
    emulator.call_input( port );
    
    if ( !emulator.call_success ) {
        Log::err << "Could not set input: " << name << " ( emulator.call_input() )\n";
        return false;
    }
    return true;
}

bool test_input_double_array( HardwareEmulator &emulator, const char *name, double *values, int size ) {
    auto port_ptr = emulator.get_port( name );
    if ( port_ptr == nullptr ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = *port_ptr;
    if ( port.buffer.type != VALUE_TYPE::DOUBLE_ARRAY ) {
        Log::err << "Type of port: " << name << " not 'double[]'\n";
        return false;
    }
	port.buffer.double_array.data = std::vector(values, values + size);
    port.buffer.double_array.size = size;
    
    emulator.call_input( port );
    
    if ( !emulator.call_success ) {
        Log::err << "Could not set input: " << name << " ( emulator.call_input() )\n";
        return false;
    }
    return true;
}

bool test_output_double( HardwareEmulator &emulator, const char *name, double &target ) {
    auto port_ptr = emulator.get_port( name );
    if ( port_ptr == nullptr ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = *port_ptr;
    if ( port.buffer.type != VALUE_TYPE::DOUBLE ) {
        Log::err << "Type of port: " << name << " not 'double'\n";
        return false;
    }
    
    emulator.call_output( port );
    if ( !emulator.call_success ) {
        Log::err << "Could not get output: " << name << " ( emulator.call_output() )\n";
        return false;
    }
    
    target = port.buffer.double_value;
    
    return true;
}


bool test_autopilot_dll() {
    EmulatorManager manager;
    if ( !manager.init( "", "" ) ) {
        Log::err << "Could not initiate EmulatorManager\n";
        return false;
    }
    
    //Setup emulator configuration
    MessageBuilder builder;
    builder.add( "autopilot", "AutopilotAdapter" );
    builder.add( "os", "windows" );
    builder.add( "test_real" );
    //builder.add( "debug", "syscalls,code,mem" );
    //builder.add( "debug", "" );
    
    auto id = manager.alloc_emulator( builder.res.c_str() );
    
    if ( id < 0 ) {
        Log::err << "Error allocating Emulator: \n";
        auto q = manager.query( "get_error_msg" );
        MessageParser p( q.c_str() );
        if ( !p.has_next() || !p.is_cmd( "error_msg" ) )
            Log::err << "Could not query 'error_msg'\n";
        else
            Log::err << p.get_string() << "\n";
        return false;
    }
    
    
    auto &emulator = *manager.emulators[id];
    
    if ( !emulator.call_success ) {
        Log::err << "Error calling init()\n";
        return false;
    }
    
    if ( !test_input_double( emulator, "timeIncrement", 1 ) )
        return false;
    if ( !test_input_double( emulator, "currentVelocity", 0 ) )
        return false;
    if ( !test_input_double( emulator, "x", 0.01 ) )
        return false;
    if ( !test_input_double( emulator, "y", 0.01 ) )
        return false;
    if ( !test_input_double( emulator, "compass", 0 ) )
        return false;
    if ( !test_input_double( emulator, "currentEngine", 0 ) )
        return false;
    if ( !test_input_double( emulator, "currentSteering", 0 ) )
        return false;
    if ( !test_input_double( emulator, "currentBrakes", 0 ) )
        return false;
    if ( !test_input_int( emulator, "trajectory_length", 5 ) )
        return false;
    double x[6] = { 0.01, 0.02, 0.03, 0.04, 0.05, 0.06 };
    double y[6] = { 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 };
    if ( !test_input_double_array( emulator, "trajectory_x", x, 6 ) )
        return false;
    if ( !test_input_double_array( emulator, "trajectory_y", y, 6 ) )
        return false;
        
    emulator.call_execute();
    if ( !emulator.call_success ) {
        Log::err << "Error calling execute()\n";
        return false;
    }
    
    double engine, steering, brakes;
    if ( !test_output_double( emulator, "engine", engine ) )
        return false;
    if ( !test_output_double( emulator, "steering", steering ) )
        return false;
    if ( !test_output_double( emulator, "brakes", brakes ) )
        return false;
        
    Log::debug << "Result: [engine=" << engine << ", steering=" << steering << ", brakes=" << brakes << "]\n";
    Log::debug << "Time: " << emulator.computer.time.micro_time << "us " << emulator.computer.time.pico_time << "ps\n";
    return true;
}

bool test_linux_elf_info() {
    FileReader fr;
    if ( fr.open( "AutopilotAdapter.so" ) ) {
        ElfFile elf;
        fr.read( elf.data );
        
        if ( !elf.parse() )
            return false;
            
            
        //elf.print();
    }
    else
        return false;
    return true;
}

bool test_simple_elf() {
    return test_simple_sample( new OS::Linux(), false );
}

bool test_funccalling_elf() {
    return test_funccalling_sample( new OS::Linux() );
}

bool test_syscall_elf() {
    return test_syscall_sample( new OS::Linux() );
}
bool test_autopilot_elf() {
    EmulatorManager manager;
    if ( !manager.init( "", "" ) ) {
        Log::err << "Could not initiate EmulatorManager\n";
        return false;
    }
    
    //Setup configuration of the emulator
    MessageBuilder builder;
    builder.add( "autopilot", "AutopilotAdapter" );
    builder.add( "os", "linux" );
    builder.add( "test_real" );
    builder.add( "cpu_frequency=10000000" );
    builder.add( "memory_frequency=1000000" );
    builder.add( "cache_DL1=128,1,2" );
    builder.add( "cache_IL1=128,1,2" );
    builder.add( "cache_L2=1024,10,15" );
    //builder.add( "debug", "code,syscalls,mem,reg_update" );
    //builder.add( "debug", "syscalls" );
    
    auto id = manager.alloc_emulator( builder.res.c_str() );
    
    if ( id < 0 ) {
        Log::err << "Error allocating Emulator: \n";
        auto q = manager.query( "get_error_msg" );
        MessageParser p( q.c_str() );
        if ( !p.has_next() || !p.is_cmd( "error_msg" ) )
            Log::err << "Could not query 'error_msg'\n";
        else
            Log::err << p.get_string() << "\n";
        return false;
    }
    
    
    auto &emulator = *manager.emulators[id];
    
    
    if ( !emulator.call_success ) {
        Log::err << "Error calling init()\n";
        return false;
    }
    
    if ( !test_input_double( emulator, "timeIncrement", 1 ) )
        return false;
    if ( !test_input_double( emulator, "currentVelocity", 0 ) )
        return false;
    if ( !test_input_double( emulator, "x", 0.01 ) )
        return false;
    if ( !test_input_double( emulator, "y", 0.01 ) )
        return false;
    if ( !test_input_double( emulator, "compass", 0 ) )
        return false;
    if ( !test_input_double( emulator, "currentEngine", 0 ) )
        return false;
    if ( !test_input_double( emulator, "currentSteering", 0 ) )
        return false;
    if ( !test_input_double( emulator, "currentBrakes", 0 ) )
        return false;
    if ( !test_input_int( emulator, "trajectory_length", 5 ) )
        return false;
    double x[6] = { 0.015, 0.02, 0.03, 0.04, 0.05, 0.06 };
    double y[6] = { 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 };
    if ( !test_input_double_array( emulator, "trajectory_x", x, 6 ) )
        return false;
    if ( !test_input_double_array( emulator, "trajectory_y", y, 6 ) )
        return false;
        
    //emulator.computer.debug.d_code = true;
    emulator.call_execute();
    if ( !emulator.call_success ) {
        Log::err << "Error calling execute()\n";
        return false;
    }
    
    emulator.execution_time = emulator.computer.time.micro_time;
    emulator.export_tick();
    
    double engine, steering, brakes;
    if ( !test_output_double( emulator, "engine", engine ) )
        return false;
    if ( !test_output_double( emulator, "steering", steering ) )
        return false;
    if ( !test_output_double( emulator, "brakes", brakes ) )
        return false;
        
    Log::debug << "Result: [engine=" << engine << ", steering=" << steering << ", brakes=" << brakes << "]\n";
    
    auto q_res = emulator.query( "get_computer_time" );
    Log::debug << q_res << "\n";
    
    return true;
}
