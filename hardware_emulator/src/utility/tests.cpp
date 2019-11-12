/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "tests.h"
#include "utility/dll_interface.h"
#include "software_simulator_manager.h"
#include "hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"
#include "utility/config.h"
#include "os_linux/elf.h"
#include "utility/utility.h"
#include "port/port_simple.h"
#include "port/port_array.h"


/*
    Tests the emulation of the functions from
    samples/simple/
    depending of the os type
*/
void test_simple_sample( OS::OS *os, bool windows ) {
    Computer computer;
    computer.init();
    computer.set_os( os );
    
    
    ADD_DLL::Interface interf;
    interf.init(computer, windows);
        
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
}


#include "../../samples/funccalling/algorithm.h"

/*
        Tests the emulation of the functions from
        samples/funccalling/
        depending of the os type

        The test compares and validates the results by computing the content of the functions
        using the algorithms from samples/funccalling/algorithm.h
*/
void test_funccalling_sample( OS::OS *os ) {
    Computer computer;
    computer.debug.debug = false;
    computer.debug.d_code = false;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    
    computer.init();
    computer.set_os( os );
    computer.os->load_file(FS::File("sample_functioncalling"));
        
    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    auto buffer_slot = section_stack.get_annotated( 1024, "Port Exchange buffer", Annotation::Type::OBJECT );
    
#define TEST_FUNCCALL(FUNC_NAME, PARAM_TYPE, COMBINE_TYPE, ...) \
    auto func_##FUNC_NAME = computer.symbols.get_symbol( #FUNC_NAME ); \
    if (func_##FUNC_NAME .type == Symbols::Symbol::Type::NONE ) { \
        throw_error(std::string("Could not find function symbol for " #FUNC_NAME "\n")); \
    } \
    computer.func_call->set_params_##PARAM_TYPE (__VA_ARGS__); \
    computer.call(func_##FUNC_NAME .addr, #FUNC_NAME); \
    auto res_##FUNC_NAME = computer.func_call->get_return_##PARAM_TYPE(); \
    auto verify_##FUNC_NAME = combine< COMBINE_TYPE >(__VA_ARGS__); \
    if ( res_##FUNC_NAME != verify_##FUNC_NAME) { \
        throw_error(std::string("Error verifying " #FUNC_NAME ":\nResult: ") + std::to_string(res_##FUNC_NAME) + " != Verify: " + std::to_string(verify_##FUNC_NAME)); \
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
    if (func_##FUNC_NAME .type == Symbols::Symbol::Type::NONE ) { \
        throw_error(std::string("Could not find function symbol for " #FUNC_NAME "\n")); \
    } \
    PARAM_TYPE local_array_##FUNC_NAME [5] = { 17, 20, 12, 7, 8 }; \
    computer.memory.write_memory(buffer_slot.start_address, 5 * sizeof( PARAM_TYPE ), (uchar *)local_array_##FUNC_NAME ); \
    computer.func_call->set_param1_64(buffer_slot.start_address); \
    computer.func_call->set_param2_32(5); \
    computer.call(func_##FUNC_NAME .addr, #FUNC_NAME); \
    auto res_##FUNC_NAME = computer.func_call->get_return_##PARAM_NAME(); \
    auto verify_##FUNC_NAME = combine_array< PARAM_TYPE >(local_array_##FUNC_NAME , 5); \
    if ( res_##FUNC_NAME != verify_##FUNC_NAME) { \
        throw_error(std::string("Error verifying " #FUNC_NAME ":\nResult: ") + std::to_string(res_##FUNC_NAME) + " != Verify: " + std::to_string(verify_##FUNC_NAME)); \
    }
    TEST_ARRAY_FUNCCALL( double_array, double, double );
    TEST_ARRAY_FUNCCALL( float_array, float, float );
    TEST_ARRAY_FUNCCALL( int_array, 32, int32_t );
    TEST_ARRAY_FUNCCALL( long_array, 64, int64_t );
    TEST_ARRAY_FUNCCALL( char_array, char, char );
    
#undef TEST_ARRAY_FUNCCALL
}




/*
    Tests the emulation of the functions from
    samples/syscall_dll/sample_syscall.dll
    samples/syscall_so/sample_syscall.so
    depending of the os type
*/
void test_syscall_sample( OS::OS *os ) {
    Computer computer;
    computer.debug.debug = false;
    computer.debug.d_code = false;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;

    /*computer.debug.debug = true;
    computer.debug.d_code = true;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = true;*/
    
    computer.init();
        
    computer.set_os( os );
    
    
    
    LOADED_DLL::Interface interf;
    interf.init(computer);
        
    /*computer.debug.debug = true;
    computer.debug.d_code = true;
    computer.debug.d_mem = true;
    computer.debug.d_reg_update = true;
    computer.debug.d_syscalls = true;*/
    Log::debug << "test_method()\n";
    interf.test_method();
    
}


void test_simple_dll() {
    test_simple_sample(new OS::Windows(), true);
}

void test_funccalling_dll() {
    test_funccalling_sample( new OS::Windows() );
}

void test_syscall_dll() {
    test_syscall_sample( new OS::Windows() );
}

void test_hardware_manager_querries() {
    SoftwareSimulatorManager manager;
    manager.init("");
    
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
        throw_error("Did not recieve response from 'get_available_threads' query to EmulatorManager\n");
    if ( !has_autopilots )
        throw_error("Did not recieve response from 'get_available_autopilots' query to EmulatorManager\n");
}

void test_input_double( HardwareEmulator &emulator, const char *name, double value ) {
    auto port_ptr = emulator.get_port( name );
    if ( port_ptr == nullptr )
        throw_error("Could not get input port id of: " + std::string(name) + "\n");
    
    auto &port = *port_ptr;
    if ( port.get_port_type() != PortType::Type::DOUBLE) throw_error("Type of port: " + std::string(name) + " not 'double'\n"); 

    ((PortDoubleEmu*)port_ptr)->data = value;
    port.process_input();
}

void test_input_int( HardwareEmulator &emulator, const char *name, int value ) {
    auto port_ptr = emulator.get_port(name);
    if (port_ptr == nullptr)
        throw_error("Could not get input port id of: " + std::string(name) + "\n");

    auto& port = *port_ptr;
    if (port.get_port_type() != PortType::Type::INT) throw_error("Type of port: " + std::string(name) + " not 'double'\n");

    ((PortIntEmu*)port_ptr)->data = value;
    port.process_input();
}

void test_input_double_array( HardwareEmulator &emulator, const char *name, double *values, int size ) {
    auto port_ptr = emulator.get_port(name);
    if (port_ptr == nullptr)
        throw_error("Could not get input port id of: " + std::string(name) + "\n");

    auto& port = *port_ptr;
    if (port.get_port_type() != PortType::Type::DOUBLE && port.get_port_dimension() != PortDimension::Dimension::ARRAY) 
        throw_error("Type of port: " + std::string(name) + " not 'double[]'\n");

    auto& data = ((PortDoubleArrayEmu*)port_ptr)->data;
    data.resize(size);
    ((PortDoubleArrayEmu*)port_ptr)->size = size;
    for (auto i : urange(size)) {
        data[i] = values[i];
    }
    port.process_input();
}

void test_output_double( HardwareEmulator &emulator, const char *name, double &target ) {
    auto port_ptr = emulator.get_port(name);
    if (port_ptr == nullptr)
        throw_error("Could not get input port id of: " + std::string(name) + "\n");

    auto& port = *port_ptr;
    if (port.get_port_type() != PortType::Type::DOUBLE) throw_error("Type of port: " + std::string(name) + " not 'double'\n");

    port.process_output();
    target = ((PortDoubleEmu*)port_ptr)->data;
}



void test_autopilot(const char* os) {
    SoftwareSimulatorManager manager;
    manager.init("");
    
    //Setup emulator configuration
    MessageBuilder builder;
    builder.add( "mode", "emu");
    builder.add("time_model", "models");
    builder.add( "software", "AutopilotAdapter" );
    builder.add( "os", os);
    builder.add( "test_real", "try" );
    builder.add( "cpu_frequency=10000000");
    builder.add( "memory_frequency=1000000");
    builder.add( "cache_DL1=128,1,2");
    builder.add( "cache_IL1=128,1,2");
    builder.add( "cache_L2=1024,10,15");
    //builder.add( "debug", "code,syscalls,mem,reg_update" );
    //builder.add( "debug", "syscalls" );
    //builder.add( "debug", "syscalls,code,mem" );
    //builder.add( "debug", "" );
    
    auto id = manager.alloc_simulator( builder.res.c_str() );
    
    auto &simulator = *((HardwareEmulator*) manager.simulators[id].get());
    
    test_input_double(simulator, "timeIncrement", 1);
    test_input_double(simulator, "currentVelocity", 0);
    test_input_double(simulator, "x", 0.01);
    test_input_double(simulator, "y", 0.01);
    test_input_double(simulator, "compass", 0);
    test_input_double(simulator, "currentEngine", 0);
    test_input_double(simulator, "currentSteering", 0);
    test_input_double(simulator, "currentBrakes", 0);
    test_input_int(simulator, "trajectory_length", 5);
    double x[6] = { 0.01, 0.02, 0.03, 0.04, 0.05, 0.06 };
    double y[6] = { 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 };
    test_input_double_array(simulator, "trajectory_x", x, 6);
    test_input_double_array(simulator, "trajectory_y", y, 6);
        
    simulator.exec();
    
    double engine, steering, brakes;
    test_output_double(simulator, "engine", engine);
    test_output_double(simulator, "steering", steering);
    test_output_double(simulator, "brakes", brakes);
        
    Log::debug << "Result: [engine=" << engine << ", steering=" << steering << ", brakes=" << brakes << "]\n";
    Log::debug << "Time: " << simulator.computer.time.micro_time << "us " << simulator.computer.time.pico_time << "ps\n";
}

void test_linux_elf_info() {
    FileReader fr;
    if (!fr.open(FS::File("AutopilotAdapter.so")))
        throw_error("Could not open AutopilotAdapter.so");
    
    ElfFile elf;
    fr.read( elf.data );
        
    if (!elf.parse())
        throw_error("Error parsing ELF.");
    //elf.print();
}

void test_simple_elf() {
    test_simple_sample( new OS::Linux(), false );
}

void test_funccalling_elf() {
    test_funccalling_sample( new OS::Linux() );
}

void test_syscall_elf() {
    test_syscall_sample( new OS::Linux() );
}

void test_autopilot_dll() {
    test_autopilot("windows");
}

void test_autopilot_elf() {
    test_autopilot("linux");
}
