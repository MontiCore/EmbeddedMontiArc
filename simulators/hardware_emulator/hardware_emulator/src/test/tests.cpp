/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "tests.h"
#include "utility/dll_interface.h"
#include "simulator/software_simulator_manager.h"
#include "simulator/hardware_emulator.h"
#include "os_windows/os_windows.h"
#include "os_linux/os_linux.h"
#include "os_linux/elf.h"
#include "utility/utility.h"
#include <iostream>

/*
    Tests the emulation of the functions from
    samples/simple/
    depending of the os type
*/
void test_simple_sample( Computer &computer, bool windows ) {
    
    
    ADD_DLL::Interface interf;
    interf.init(computer, windows);

    if (do_debug) {
        computer.debug.debug = true;
        //computer.debug.d_code = true;
        //computer.debug.d_mem = true;
        //computer.debug.d_regs = true;
        //computer.debug.d_reg_update = true;
        //computer.debug.d_syscalls = true;
        computer.debug.d_call = true;
        computer.debug.d_unsupported_syscalls = true;
    }
    Log::debug.log_tag("add(2,3):\n");
    auto res = interf.add( 2, 3 );
    Log::debug.log_tag("Result=%d\n", res);
}


#include "../../samples/funccalling/algorithm.h"

/*
        Tests the emulation of the functions from
        samples/funccalling/
        depending of the os type

        The test compares and validates the results by computing the content of the functions
        using the algorithms from samples/funccalling/algorithm.h
*/
void test_funccalling_sample_windows() {
    Computer computer;
    if (do_debug) {
        computer.debug.debug = false;
        computer.debug.d_code = false;
        computer.debug.d_mem = false;
        computer.debug.d_regs = false;
        computer.debug.d_reg_update = false;
        computer.debug.d_syscalls = false;
    }
    
    computer.init();
    computer.set_os( new OS::Windows(computer.func_call_windows) );
    computer.os->load_file(fs::path("sample_functioncalling"));
        
    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    auto buffer_slot = section_stack.get_annotated( 1024, "Port Exchange buffer", Annotation::Type::OBJECT );
    
#define TEST_FUNCCALL(FUNC_NAME, PARAM_TYPE, COMBINE_TYPE, ...) \
    auto func_##FUNC_NAME = computer.symbols.get_symbol( #FUNC_NAME ); \
    if (func_##FUNC_NAME .type == Symbols::Symbol::Type::NONE ) { \
        throw_error(std::string("Could not find function symbol for " #FUNC_NAME "\n")); \
    } \
    computer.func_call_windows.set_params_##PARAM_TYPE (__VA_ARGS__); \
    computer.call(func_##FUNC_NAME .addr, #FUNC_NAME); \
    auto res_##FUNC_NAME = computer.func_call_windows.get_return_##PARAM_TYPE(); \
    auto verify_##FUNC_NAME = combine< COMBINE_TYPE >(__VA_ARGS__); \
    if ( res_##FUNC_NAME != verify_##FUNC_NAME) { \
        throw_error(std::string("Error verifying " #FUNC_NAME ":\nResult: ") + std::to_string(res_##FUNC_NAME) + " != Verify: " + std::to_string(verify_##FUNC_NAME)); \
    }

    
    
    TEST_FUNCCALL( int_one, 32, int32_t, 24 );
    TEST_FUNCCALL( int_two, 32, int32_t, 24, 13 );
    TEST_FUNCCALL( int_three, 32, int32_t, 24, 13, 11 );
    TEST_FUNCCALL( int_four, 32, int32_t, 24, 13, 11, 31 );
    
    
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
    computer.func_call_windows.set_param1_64(buffer_slot.start_address); \
    computer.func_call_windows.set_param2_32(5); \
    computer.call(func_##FUNC_NAME .addr, #FUNC_NAME); \
    auto res_##FUNC_NAME = computer.func_call_windows.get_return_##PARAM_NAME(); \
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

    /*computer.debug.debug = true;
    computer.debug.d_code = true;*/
    //computer.debug.d_mem = true;
    //computer.debug.d_reg_update = true;
    /*computer.debug.d_syscalls = true;
    computer.debug.d_call = true;*/

    auto func_test_long_function_intern = computer.symbols.get_symbol("test_long_function_intern"); \
    if (func_test_long_function_intern.type == Symbols::Symbol::Type::NONE ) { \
        throw_error(std::string("Could not find function symbol for 'test_long_function_intern'\n"));
    }
    computer.call(func_test_long_function_intern.addr, "test_long_function_intern"); \
    auto res_test_long_function_intern = computer.func_call_windows.get_return_32();
    std::cout << "res_test_long_function_intern= " << res_test_long_function_intern << std::endl;
}

/*
        Tests the emulation of the functions from
        samples/funccalling/
        depending of the os type

        The test compares and validates the results by computing the content of the functions
        using the algorithms from samples/funccalling/algorithm.h
*/
void test_funccalling_sample_linux() {
    Computer computer;
    if (do_debug) {
        computer.debug.debug = false;
        computer.debug.d_code = false;
        computer.debug.d_mem = false;
        computer.debug.d_regs = false;
        computer.debug.d_reg_update = false;
        computer.debug.d_syscalls = false;
    }
    
    computer.init();
    computer.set_os( new OS::Linux(computer.func_call_linux) );
    computer.os->load_file(fs::path("sample_functioncalling"));

    auto &section = computer.memory.sys_section;
    auto &section_stack = computer.memory.sys_section_stack;
    auto buffer_slot = section_stack.get_annotated( 1024, "Port Exchange buffer", Annotation::Type::OBJECT );
    
#define TEST_FUNCCALL(FUNC_NAME, PARAM_TYPE, COMBINE_TYPE, ...) \
    auto func_##FUNC_NAME = computer.symbols.get_symbol( #FUNC_NAME ); \
    if (func_##FUNC_NAME .type == Symbols::Symbol::Type::NONE ) { \
        throw_error(std::string("Could not find function symbol for " #FUNC_NAME "\n")); \
    } \
    computer.func_call_linux.set_params_##PARAM_TYPE (__VA_ARGS__); \
    computer.call(func_##FUNC_NAME .addr, #FUNC_NAME); \
    auto res_##FUNC_NAME = computer.func_call_linux.get_return_##PARAM_TYPE(); \
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
    computer.func_call_linux.set_param1_64(buffer_slot.start_address); \
    computer.func_call_linux.set_param2_32(5); \
    computer.call(func_##FUNC_NAME .addr, #FUNC_NAME); \
    auto res_##FUNC_NAME = computer.func_call_linux.get_return_##PARAM_NAME(); \
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
void test_syscall_sample( Computer &computer ) {
    if (do_debug) {
        /*computer.debug.debug = true;
        computer.debug.d_code = true;
        computer.debug.d_mem = false;
        computer.debug.d_regs = false;
        computer.debug.d_reg_update = false;
        computer.debug.d_syscalls = true;*/
    }

    
    
    
    
    LOADED_DLL::Interface interf;
    interf.init(computer);
        
    /*computer.debug.debug = true;
    computer.debug.d_code = true;
    computer.debug.d_mem = true;
    computer.debug.d_reg_update = true;
    computer.debug.d_syscalls = true;*/
    Log::debug.log_tag("test_method()\n");
    interf.test_method();
    
}


void test_simple_dll() {

    Computer computer;
    computer.init();
    computer.set_os( new OS::Windows(computer.func_call_windows) );
    test_simple_sample(computer, true);
}

void test_funccalling_dll() {
    test_funccalling_sample_windows();
}

void test_syscall_dll() {
    Computer computer;
    computer.init();
    computer.set_os( new OS::Windows(computer.func_call_windows) );
    test_syscall_sample( computer );
}

void test_hardware_manager_querries() {
    SoftwareSimulatorManager manager;
    manager.init("");
    // TODO
    // MessageBuilder builder;
    // builder.add( "get_available_autopilots" );
    // builder.add( "get_available_threads" );
    // auto query_res = manager.query( builder.res.c_str() );
    // MessageParser parser( query_res.c_str() );
    // bool has_threads = false;
    // bool has_autopilots = false;
    
    // while ( parser.has_next() ) {
    //     if ( parser.is_cmd( "available_threads" ) ) {
    //         slong thread_count;
    //         if ( !parser.get_long( thread_count ) )
    //             Log::err << "Did not recieve integer from 'available_threads' query to EmulatorManager\n";
    //         else {
    //             Log::debug << "Available threads: " << thread_count << '\n';
    //             has_threads = true;
    //         }
    //     }
    //     else if ( parser.is_cmd( "available_autopilots" ) ) {
    //         auto autopilots = parser.get_string();
    //         if ( autopilots.size() == 0 ) {
    //             //Should at least contain the dlls used for earlier tests
    //             Log::err << "Did not recieve string from 'get_available_autopilots' query to EmulatorManager\n";
    //         }
    //         else {
    //             Log::debug << "Available autopilots:\n" << autopilots << '\n';
    //             has_autopilots = true;
    //         }
    //     }
    // }
    
    // if ( !has_threads )
    //     throw_error("Did not recieve response from 'get_available_threads' query to EmulatorManager\n");
    // if ( !has_autopilots )
    //     throw_error("Did not recieve response from 'get_available_autopilots' query to EmulatorManager\n");
}

void test_zigzag_autopilot(const char* config_str, bool is_emu){
    auto config = json::parse(config_str);
    SoftwareSimulatorManager manager;
    manager.init(json());

    auto id = manager.alloc_simulator(config);
    auto &simulator = *(manager.simulators[id].get());
    auto &prog = *(simulator.program_functions.get());

    if (is_emu) {
        auto& computer = ((HardwareEmulator*)manager.simulators[id].get())->computer;
        if (do_debug) {
            computer.debug.debug = true;
            //computer.debug.d_code = true;
            //computer.debug.d_mem = true;
            //computer.debug.d_reg_update = true;
            //computer.debug.d_syscalls = true;
            computer.debug.d_call = true;
            //computer.debug.d_unsupported_syscalls = true;
        }
        else {
            computer.debug.debug = false;
        }
    }

    auto true_velocity_id = simulator.get_port_id_expected(std::string("true_velocity"));
    auto true_position_id = simulator.get_port_id_expected(std::string("true_position"));
    auto true_compass_id = simulator.get_port_id_expected(std::string("true_compass"));
    auto trajectory_length_id = simulator.get_port_id(std::string("trajectory_length"));
    auto trajectory_x_id = simulator.get_port_id_expected(std::string("trajectory_x"));
    auto trajectory_y_id = simulator.get_port_id_expected(std::string("trajectory_y"));
    auto set_gas_id = simulator.get_port_id_expected(std::string("set_gas"));
    auto set_steering_id = simulator.get_port_id_expected(std::string("set_steering"));
    auto set_braking_id = simulator.get_port_id_expected(std::string("set_braking"));


    prog.set_port(true_velocity_id, "5.6", 1);
    prog.set_port(true_position_id, "[0.0,0.0]", 1);
    prog.set_port(true_compass_id, "0.0", 1);
    if (trajectory_length_id >= 0) {
        prog.set_port(trajectory_length_id, "4", 1);
    }
    prog.set_port(trajectory_x_id, "[3, 0.0, 1.0, 2.0]", 1);
    prog.set_port(trajectory_y_id, "[3,0.0,0.0,-1.0]", 1);

    prog.execute(0.1);

    
    std::string gas = prog.get_port(set_gas_id, 1);
    std::string steering = prog.get_port(set_steering_id, 1);
    std::string brakes = prog.get_port(set_braking_id, 1);
    

    Log::debug.log_tag("Result (raw strings): [gas=%s, steering=%s, brakes=%s]\n", gas.c_str(), steering.c_str(), brakes.c_str());
    Log::debug.log_tag("Result (parsed):      [gas=%LF, steering=%LF, brakes=%LF]\n", std::stod(gas), std::stod(steering), std::stod(brakes));
    //Log::debug << "Time: " << simulator.computer.time.micro_time << "us " << simulator.computer.time.pico_time << "ps\n";
}


void test_ema_autopilot(const char* config_str, bool is_emu) {
    auto config = json::parse(config_str);
    SoftwareSimulatorManager manager;
    manager.init(json());

    auto id = manager.alloc_simulator(config);
    auto& simulator = *(manager.simulators[id].get());
    auto& prog = *(simulator.program_functions.get());

    if (is_emu) {
        auto& computer = ((HardwareEmulator*)manager.simulators[id].get())->computer;
        if (do_debug) {
            computer.debug.debug = true;
            //computer.debug.d_code = true;
            //computer.debug.d_mem = true;
            //computer.debug.d_reg_update = true;
            //computer.debug.d_syscalls = true;
            computer.debug.d_call = true;
            //computer.debug.d_unsupported_syscalls = true;
        }
        else {
            computer.debug.debug = false;
        }
    }

    auto true_velocity_id = simulator.get_port_id_expected(std::string("true_velocity"));
    auto true_position_id = simulator.get_port_id_expected(std::string("true_position"));
    auto true_compass_id = simulator.get_port_id_expected(std::string("true_compass"));
    auto trajectory_length_id = simulator.get_port_id(std::string("trajectory_length"));
    auto trajectory_x_id = simulator.get_port_id_expected(std::string("trajectory_x"));
    auto trajectory_y_id = simulator.get_port_id_expected(std::string("trajectory_y"));
    auto set_gas_id = simulator.get_port_id_expected(std::string("set_gas"));
    auto set_steering_id = simulator.get_port_id_expected(std::string("set_steering"));
    auto set_braking_id = simulator.get_port_id_expected(std::string("set_braking"));


    prog.set_port(true_velocity_id, "5.6", 1);
    prog.set_port(true_position_id, "[0.0,0.0]", 1);
    prog.set_port(true_compass_id, "0.0", 1);
    if (trajectory_length_id >= 0) {
        prog.set_port(trajectory_length_id, "4", 1);
    }
    prog.set_port(trajectory_x_id, "[10, 3, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]", 1);
    prog.set_port(trajectory_y_id, "[10, 3, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]", 1);

    prog.execute(0.1);


    std::string gas = prog.get_port(set_gas_id, 1);
    std::string steering = prog.get_port(set_steering_id, 1);
    std::string brakes = prog.get_port(set_braking_id, 1);


    Log::debug.log_tag("Result (raw strings): [gas=%s, steering=%s, brakes=%s]\n", gas.c_str(), steering.c_str(), brakes.c_str());
    Log::debug.log_tag("Result (parsed):      [gas=%LF, steering=%LF, brakes=%LF]\n", std::stod(gas), std::stod(steering), std::stod(brakes));
    //Log::debug << "Time: " << simulator.computer.time.micro_time << "us " << simulator.computer.time.pico_time << "ps\n";
}



void test_zigzag_autopilot_native() {
    test_zigzag_autopilot(R"(
    {
  "type": "computer",
  "name": "UnnamedComponent",
  "connected_to": [],
  "priority": {},
  "software_name": "zigzag_autopilot_lib",
  "backend": {
    "type": "direct"
  },
  "time_model": {
    "type": "models",
    "cpu_frequency": 4000000000,
    "memory_frequency": 2500000000,
    "caches": [
      {
        "type": "I",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "D",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "shared",
        "level": 2,
        "size": 2097152,
        "read_ticks": 6,
        "write_ticks": 6
      },
      {
        "type": "shared",
        "level": 3,
        "size": 16777216,
        "read_ticks": 40,
        "write_ticks": 40
      }
    ]
  },
  "cycle_duration": [
    0,
    20000000
  ],
  "debug_flags": []
}
    )", false);
}

void test_zigzag_autopilot_emu_windows() {
    test_zigzag_autopilot(R"(
    {
        "software_name": "zigzag_autopilot_lib",
        "backend": {
            "type": "emu",
            "os": "windows"
        },
        "time_model": {
            "type": "models",
            "cpu_frequency": 4000000000,
            "memory_frequency": 2500000000,
            "caches": [
                {"type": "I", "level": 1, "size": 262144, "read_ticks": 4, "write_ticks": 4},
                {"type": "D", "level": 1, "size": 262144, "read_ticks": 4, "write_ticks": 4},
                {"type": "shared", "level": 2, "size": 2097152, "read_ticks": 6, "write_ticks": 6},
                {"type": "shared", "level": 3, "size": 16777216, "read_ticks": 40, "write_ticks": 40}
            ]
        },
        "debug_flags": ["p_unsupported_syscalls"]
    }
    )", true);
}
void test_zigzag_autopilot_emu_linux() {
    test_zigzag_autopilot(R"(
    {
  "type": "computer",
  "name": "UnnamedComponent",
  "connected_to": [],
  "priority": {},
  "software_name": "zigzag_autopilot_lib",
  "backend": {
    "type": "emu",
    "os": "linux"
  },
  "time_model": {
    "type": "models",
    "cpu_frequency": 4000000000,
    "memory_frequency": 2500000000,
    "caches": [
      {
        "type": "I",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "D",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "shared",
        "level": 2,
        "size": 2097152,
        "read_ticks": 6,
        "write_ticks": 6
      },
      {
        "type": "shared",
        "level": 3,
        "size": 16777216,
        "read_ticks": 40,
        "write_ticks": 40
      }
    ]
  },
  "cycle_duration": [
    0,
    20000000
  ],
  "debug_flags": ["p_unsupported_syscalls", "p_call"]
}
    )", true);
}


void test_ema_autopilot_native() {
    test_ema_autopilot(R"(
    {
  "type": "computer",
  "name": "UnnamedComponent",
  "connected_to": [],
  "priority": {},
  "software_name": "ema_autopilot_lib",
  "backend": {
    "type": "direct"
  },
  "time_model": {
    "type": "models",
    "cpu_frequency": 4000000000,
    "memory_frequency": 2500000000,
    "caches": [
      {
        "type": "I",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "D",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "shared",
        "level": 2,
        "size": 2097152,
        "read_ticks": 6,
        "write_ticks": 6
      },
      {
        "type": "shared",
        "level": 3,
        "size": 16777216,
        "read_ticks": 40,
        "write_ticks": 40
      }
    ]
  },
  "cycle_duration": [
    0,
    20000000
  ],
  "debug_flags": []
}
    )", false);
}

void test_ema_autopilot_emu_windows() {
    test_ema_autopilot(R"(
    {
        "software_name": "ema_autopilot_lib",
        "backend": {
            "type": "emu",
            "os": "windows"
        },
        "time_model": {
            "type": "models",
            "cpu_frequency": 4000000000,
            "memory_frequency": 2500000000,
            "caches": [
                {"type": "I", "level": 1, "size": 262144, "read_ticks": 4, "write_ticks": 4},
                {"type": "D", "level": 1, "size": 262144, "read_ticks": 4, "write_ticks": 4},
                {"type": "shared", "level": 2, "size": 2097152, "read_ticks": 6, "write_ticks": 6},
                {"type": "shared", "level": 3, "size": 16777216, "read_ticks": 40, "write_ticks": 40}
            ]
        },
        "debug_flags": ["p_unsupported_syscalls", "p_call" ]
    }
    )", true);
}
void test_ema_autopilot_emu_linux() {
    test_ema_autopilot(R"(
    {
  "type": "computer",
  "name": "UnnamedComponent",
  "connected_to": [],
  "priority": {},
  "software_name": "ema_autopilot_lib",
  "backend": {
    "type": "emu",
    "os": "linux"
  },
  "time_model": {
    "type": "models",
    "cpu_frequency": 4000000000,
    "memory_frequency": 2500000000,
    "caches": [
      {
        "type": "I",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "D",
        "level": 1,
        "size": 262144,
        "read_ticks": 4,
        "write_ticks": 4
      },
      {
        "type": "shared",
        "level": 2,
        "size": 2097152,
        "read_ticks": 6,
        "write_ticks": 6
      },
      {
        "type": "shared",
        "level": 3,
        "size": 16777216,
        "read_ticks": 40,
        "write_ticks": 40
      }
    ]
  },
  "cycle_duration": [
    0,
    20000000
  ],
  "debug_flags": ["p_call", "p_syscalls"]
}
    )", true);
}

void test_linux_elf_info() {
    FileReader fr;
    if (!fr.open(fs::path("zigzag_autopilot_lib.so")))
        throw_error("Could not open zigzag_autopilot_lib.so");
    
    ElfFile elf;
    fr.read( elf.data );
        
    if (!elf.parse())
        throw_error("Error parsing ELF.");
    //elf.print();
}

void test_simple_elf() {

    Computer computer;
    computer.init();
    computer.set_os( new OS::Linux(computer.func_call_linux) );
    test_simple_sample( computer, false );
}

void test_funccalling_elf() {
    test_funccalling_sample_linux( );
}

void test_syscall_elf() {
    Computer computer;
    computer.init();
    computer.set_os( new OS::Linux(computer.func_call_linux) );
    test_syscall_sample( computer );
}
