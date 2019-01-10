#include "autopilot_interface.h"
#include "utility.h"
#include "os_windows/windows_calls.h"


using namespace std;

AutopilotInterface AutopilotInterface::instance;




bool AutopilotInterface::test_main() {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    Log::debug << "AutopilotInterface::init()\n";
    if ( !init() )
        return false;
    Log::debug << "init(0,0)\n";
    emulator.call( AutopilotInterface::INIT );
    
    Log::debug << "set_compass(0.0)\n";
    emulator.set_input_double( SET_VELOCITY, 0.0 );
    Log::debug << "set_currentBrakes(0.0)\n";
    emulator.set_input_double( SET_BRAKES, 0.0 );
    Log::debug << "currentEngine(0.0)\n";
    emulator.set_input_double( SET_ENGINE, 0.0 );
    Log::debug << "currentSteering(0.0)\n";
    emulator.set_input_double( SET_STEERING, 0.0 );
    Log::debug << "currentVelocity(0.0)\n";
    emulator.set_input_double( SET_VELOCITY, 0.0 );
    Log::debug << "set_timeIncrement(1)\n";
    emulator.set_input_double( SET_TIME_INC, 1.0 );
    Log::debug << "set_trajectory_length(5)\n";
    emulator.set_input_int( SET_TRAJECTORY_LENGTH, 5 );
    double x[6] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
    double y[6] = { 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 };
    Log::debug << "set_trajectory_x({0.01, 0.02, 0.03, 0.04, 0.05, 0.06}, 6)\n";
    emulator.set_input_array( SET_TRAJECTORY_X, x, 6 );
    Log::debug << "set_trajectory_y({ 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 }, 6)\n";
    emulator.set_input_array( SET_TRAJECTORY_Y, y, 6 );
    Log::debug << "set_x(0.01)\n";
    emulator.set_input_double( SET_X, 0.01 );
    Log::debug << "set_y(0.01)\n";
    emulator.set_input_double( SET_Y, 0.01 );
    
    Log::debug << "exec()\n";
    emulator.simulation_time = 1000000000L;
    exec();
    
    Log::debug << "jni_get_brakes()=";
    double brakes = emulator.get_output( GET_BRAKES );
    Log::debug << to_string( brakes ) << "\n";
    Log::debug << "jni_get_engine()=";
    double engine = emulator.get_output( GET_ENGINE );
    Log::debug << to_string( engine ) << "\n";
    Log::debug << "jni_get_steering()=";
    double steering = emulator.get_output( GET_STEERING );
    Log::debug << to_string( steering ) << "\n";
    
    return emulator.call_success;
}

bool AutopilotInterface::init() {
    //computer.debug.debug = false;
    computer.init();
    if ( !computer.loaded() )
        return false;
        
    const char *file_name = "AutopilotModel.dll";
    const char *module_name = "AutopilotAdapter.dll";
    
    os_windows.init( computer );
    WindowsCalls::add_windows_calls( computer.sys_calls, os_windows );
    Log::debug << "Load DLL AutopilotModel.dll\n";
    if ( !os_windows.load_dll( file_name ) )
        return false;
        
    emulator.init( computer, module_name );
    emulator.set_function_count( FUNCTION_COUNT );
    emulator.set_input_count( INPUT_FUNC_COUNT );
    emulator.set_output_count( OUTPUT_FUNC_COUNT );
    
    emulator.reg_function(
        SET_TIME_INC,
        "Java_simulator_integration_AutopilotAdapter_set_1timeIncrement",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_VELOCITY,
        "Java_simulator_integration_AutopilotAdapter_set_1currentVelocity",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_X,
        "Java_simulator_integration_AutopilotAdapter_set_1x",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_Y,
        "Java_simulator_integration_AutopilotAdapter_set_1y",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_COMPASS,
        "Java_simulator_integration_AutopilotAdapter_set_1compass",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_ENGINE,
        "Java_simulator_integration_AutopilotAdapter_set_1currentEngine",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_STEERING,
        "Java_simulator_integration_AutopilotAdapter_set_1currentSteering",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_BRAKES,
        "Java_simulator_integration_AutopilotAdapter_set_1currentBrakes",
        HardwareEmulator::INPUT_DOUBLE
    );
    emulator.reg_function(
        SET_TRAJECTORY_LENGTH,
        "Java_simulator_integration_AutopilotAdapter_set_1trajectory_1length",
        HardwareEmulator::INPUT_INT
    );
    emulator.reg_function(
        SET_TRAJECTORY_X,
        "Java_simulator_integration_AutopilotAdapter_set_1trajectory_1x",
        HardwareEmulator::INPUT_ARRAY
    );
    emulator.reg_function(
        SET_TRAJECTORY_Y,
        "Java_simulator_integration_AutopilotAdapter_set_1trajectory_1y",
        HardwareEmulator::INPUT_ARRAY
    );
    emulator.reg_function(
        GET_ENGINE,
        "Java_simulator_integration_AutopilotAdapter_get_1engine",
        HardwareEmulator::OUTPUT
    );
    emulator.reg_function(
        GET_STEERING,
        "Java_simulator_integration_AutopilotAdapter_get_1steering",
        HardwareEmulator::OUTPUT
    );
    emulator.reg_function(
        GET_BRAKES,
        "Java_simulator_integration_AutopilotAdapter_get_1brakes",
        HardwareEmulator::OUTPUT
    );
    emulator.reg_function(
        EXEC,
        "Java_simulator_integration_AutopilotAdapter_exec",
        0
    );
    emulator.reg_function(
        INIT,
        "Java_simulator_integration_AutopilotAdapter_init",
        0
    );
    emulator.reg_function(
        MESSAGE,
        "Java_simulator_integration_AutopilotAdapter_message",
        0
    );
    loaded = true;
    return true;
}

void AutopilotInterface::exec() {
    if ( !emulator.computing() ) {
        for ( uint i : Range( ( uint )INPUT_FUNC_COUNT ) ) {
            if ( emulator.has_new_input( i ) )
                emulator.call( i );
        }
        emulator.call( EXEC );
        for ( uint i : Range( ( uint )OUTPUT_FUNC_START, ( uint )OUTPUT_FUNC_END ) )
            emulator.call( i );
    }
}

bool str_equal( const char *first, uint size, const char *second ) {
    for ( uint i : Range( size ) )
        if ( second[i] == '\0' || second[i] != first[i] )
            return false;
    return second[size] == '\0';
}

std::string AutopilotInterface::message( const char *msg ) {
    MessageParser parser( msg );
    if ( !parser.has_cmd )
        return "false";
        
    if ( parser.is_cmd( "inc" ) ) {
        slong time_delta;
        if ( !parser.get_long( time_delta ) )
            return "err=Did not find long parameter";
        emulator.add_time( time_delta );
        Log::debug << Log::tag << "Time 'inc' msg recieved: " << time_delta << "\n";
        return "true";
    }
    else if ( parser.is_cmd( "get_computer_time" ) )
        return "res=" + to_string( computer.computing_time );
    else
        Log::debug << Log::tag << "Unkown message recieved: " << msg << "\n";
        
        
    return "err=Unknown Command";
}

MessageParser::MessageParser( const char *msg ) {
    this->msg = msg;
    pos = 0;
    while ( msg[pos] != 0 && msg[pos] != '=' )
        ++pos;
    if ( pos == 0 ) {
        this->has_cmd = false;
        return;
    }
    this->has_cmd = true;
    
    this->cmd_size = pos;
    this->rest = msg + pos + 1;
}

bool MessageParser::is_cmd( const char *cmd ) {
    return str_equal( msg, cmd_size, cmd );
}

bool MessageParser::get_long( slong &target ) {
    const char *new_ptr;
    ulong val = strtoll( rest, ( char ** )&new_ptr, 10 );
    if ( new_ptr == rest )
        return false;
    target = val;
    rest = new_ptr;
    to_comma();
    if ( *rest != '\0' )
        ++rest;
    return true;
}

void MessageParser::to_non_ws() {
    while ( *rest != '\0' && iswspace( *rest ) )
        ++rest;
}

void MessageParser::to_comma() {
    while ( *rest != '\0' && *rest != ',' )
        ++rest;
}
