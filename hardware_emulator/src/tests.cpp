#include "tests.h"
#include "dll_interface.h"
#include "emulator/emulator_manager.h"
#include "os_windows/windows_calls.h"
#include "config.h"
#include "os_linux/elf.h"

bool test_simple_dll() {
    ADD_DLL::DllInterface interf;
    interf.computer.debug.debug = true;
    interf.computer.debug.d_mem = false;
    interf.computer.debug.d_regs = false;
    interf.computer.debug.d_reg_update = false;
    interf.computer.debug.d_syscalls = false;
    
    interf.computer.init();
    if ( !interf.computer.loaded() )
        return false;
        
        
    interf.os_windows.init( interf.computer );
    WindowsCalls calls;
    calls.add_windows_calls( interf.computer.sys_calls, interf.os_windows );
    if ( !interf.os_windows.load_file( "SampleDLL.dll" ) )
        return false;
        
        
    interf.init();
    Log::debug << "add(2,3):\n";
    auto res = interf.add( 2, 3 );
    Log::debug << "Result=" << res << "\n";
    
    return interf.call_success;
}

bool test_syscall_dll() {
    LOADED_DLL::DllInterface interf;
    interf.computer.debug.debug = true;
    interf.computer.debug.d_mem = false;
    interf.computer.debug.d_regs = false;
    interf.computer.debug.d_reg_update = false;
    interf.computer.debug.d_syscalls = false;
    
    interf.computer.init();
    if ( !interf.computer.loaded() )
        return false;
    interf.os_windows.init( interf.computer );
    
    
    WindowsCalls calls;
    calls.add_windows_calls( interf.computer.sys_calls, interf.os_windows );
    if ( !interf.os_windows.load_file( "loaded_dll.dll" ) )
        return false;
        
        
    interf.init();
    Log::debug << "test_method()\n";
    interf.test_method();
    
    return interf.call_success;
}

bool test_hardware_manager_querries() {
    EmulatorManager manager;
    if ( !manager.init() ) {
        Log::err << "Could not initiate EmulatorManager\n";
        return false;
    }
    
    MessageBuilder builder;
    builder.add( "get_available_autopilots" );
    builder.add( "get_available_threads" );
    auto querry_res = manager.querry( builder.res.c_str() );
    MessageParser parser( querry_res.c_str() );
    bool has_threads = false;
    bool has_autopilots = false;
    
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "available_threads" ) ) {
            slong thread_count;
            if ( !parser.get_long( thread_count ) )
                Log::err << "Did not recieve integer from 'available_threads' querry to EmulatorManager\n";
            else {
                Log::debug << "Available threads: " << thread_count << '\n';
                has_threads = true;
            }
        }
        else if ( parser.is_cmd( "available_autopilots" ) ) {
            auto autopilots = parser.get_string();
            if ( autopilots.size() == 0 ) {
                //Should at least contain the dlls used for earlier tests
                Log::err << "Did not recieve string from 'get_available_autopilots' querry to EmulatorManager\n";
            }
            else {
                Log::debug << "Available autopilots:\n" << autopilots << '\n';
                has_autopilots = true;
            }
        }
    }
    
    if ( !has_threads )
        Log::err << "Did not recieve response from 'get_available_threads' querry to EmulatorManager\n";
    if ( !has_autopilots )
        Log::err << "Did not recieve response from 'get_available_autopilots' querry to EmulatorManager\n";
        
    if ( !has_threads || !has_autopilots )
        return false;
        
    return true;
}

bool test_input_double( HardwareEmulator &emulator, const char *name, double value ) {
    auto func_id = emulator.get_port_id( name );
    if ( func_id == -1 ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = emulator.input_ports[func_id];
    if ( port.buffer.type != VALUE_TYPE::DOUBLE ) {
        Log::err << "Type of port: " << name << " not 'double'\n";
        return false;
    }
    port.buffer.double_value = value;
    
    emulator.call_input( func_id );
    
    if ( !emulator.call_success ) {
        Log::err << "Could not set input: " << name << " ( emulator.call_input() )\n";
        return false;
    }
    return true;
}

bool test_input_int( HardwareEmulator &emulator, const char *name, int value ) {
    auto func_id = emulator.get_port_id( name );
    if ( func_id == -1 ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = emulator.input_ports[func_id];
    if ( port.buffer.type != VALUE_TYPE::INT ) {
        Log::err << "Type of port: " << name << " not 'int'\n";
        return false;
    }
    port.buffer.int_value = value;
    
    emulator.call_input( func_id );
    
    if ( !emulator.call_success ) {
        Log::err << "Could not set input: " << name << " ( emulator.call_input() )\n";
        return false;
    }
    return true;
}

bool test_input_double_array( HardwareEmulator &emulator, const char *name, double *values, int size ) {
    auto func_id = emulator.get_port_id( name );
    if ( func_id == -1 ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = emulator.input_ports[func_id];
    if ( port.buffer.type != VALUE_TYPE::DOUBLE_ARRAY ) {
        Log::err << "Type of port: " << name << " not 'double[]'\n";
        return false;
    }
    port.buffer.double_array.data.init( size, values );
    port.buffer.double_array.size = size;
    
    emulator.call_input( func_id );
    
    if ( !emulator.call_success ) {
        Log::err << "Could not set input: " << name << " ( emulator.call_input() )\n";
        return false;
    }
    return true;
}

bool test_output_double( HardwareEmulator &emulator, const char *name, double &target ) {
    auto func_id = emulator.get_port_id( name );
    if ( func_id == -1 ) {
        Log::err << "Could not get input port id of: " << name << "\n";
        return false;
    }
    auto &port = emulator.output_ports[func_id];
    if ( port.buffer.type != VALUE_TYPE::DOUBLE ) {
        Log::err << "Type of port: " << name << " not 'double'\n";
        return false;
    }
    
    emulator.call_output( func_id );
    if ( !emulator.call_success ) {
        Log::err << "Could not get output: " << name << " ( emulator.call_output() )\n";
        return false;
    }
    
    target = port.buffer.double_value;
    
    return true;
}

bool test_autopilot_dll() {
    EmulatorManager manager;
    if ( !manager.init() ) {
        Log::err << "Could not initiate EmulatorManager\n";
        return false;
    }
    
    MessageBuilder builder;
    builder.add( "autopilot", "AutopilotAdapter" );
    builder.add( "os", "windows" );
    //builder.add( "debug", "" );
    
    auto id = manager.alloc_emulator( builder.res.c_str() );
    
    if ( id < 0 ) {
        Log::err << "Error allocating Emulator: \n";
        auto q = manager.querry( "get_error_msg" );
        MessageParser p( q.c_str() );
        if ( !p.has_next() || !p.is_cmd( "error_msg" ) )
            Log::err << "Could not querry 'error_msg'\n";
        else
            Log::err << p.get_string() << "\n";
        return false;
    }
    
    
    auto &emulator = *manager.emulators[id];
    
    
    emulator.call_void( emulator.init_address );
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
        
    emulator.call_void( emulator.execute_address );
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
    
    return true;
}

bool test_linux_elf_info() {
    FileReader fr;
    if ( fr.open( "linux-exec" ) ) {
        ElfFile elf;
        fr.read( elf.data );
        
        if ( !elf.parse() )
            return false;
            
            
        //elf.print();
    }
    else
        return false;
    if ( fr.open( "libLinux.so" ) ) {
        ElfFile elf;
        fr.read( elf.data );
        
        if ( !elf.parse() )
            return false;
            
            
        elf.print();
    }
    else
        return false;
    return true;
}
