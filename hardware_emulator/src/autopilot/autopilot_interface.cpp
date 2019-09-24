/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
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
    emulator.call_void( emulator.init_address );
    
    
    /*
        { "timeIncrement", VALUE_TYPE::DOUBLE },    //0
        { "currentVelocity", VALUE_TYPE::DOUBLE },  //1
        { "x", VALUE_TYPE::DOUBLE },                //2
        { "y", VALUE_TYPE::DOUBLE },                //3
        { "compass", VALUE_TYPE::DOUBLE },          //4
        { "currentEngine", VALUE_TYPE::DOUBLE },    //5
        { "currentSteering", VALUE_TYPE::DOUBLE },  //6
        { "currentBrakes", VALUE_TYPE::DOUBLE },    //7
        { "trajectory_length", VALUE_TYPE::INT },   //8
        { "trajectory_x", VALUE_TYPE::DOUBLE_ARRAY },   //9
        { "trajectory_y", VALUE_TYPE::DOUBLE_ARRAY }    //10
    */
    Log::debug << "set_timeIncrement(1)\n";
    emulator.get_input( 0 ).init( 1 );
    emulator.call_input( 0 );
    Log::debug << "currentVelocity(0.0)\n";
    emulator.get_input( 1 ).init( 0.0 );
    emulator.call_input( 1 );
    Log::debug << "set_x(0.01)\n";
    emulator.get_input( 2 ).init( 0.01 );
    emulator.call_input( 2 );
    Log::debug << "set_y(0.01)\n";
    emulator.get_input( 3 ).init( 0.01 );
    emulator.call_input( 3 );
    Log::debug << "set_compass(0.0)\n";
    emulator.get_input( 4 ).init( 0.0 );
    emulator.call_input( 4 );
    Log::debug << "currentEngine(0.0)\n";
    emulator.get_input( 5 ).init( 0.0 );
    emulator.call_input( 5 );
    Log::debug << "currentSteering(0.0)\n";
    emulator.get_input( 6 ).init( 0.0 );
    emulator.call_input( 6 );
    Log::debug << "set_currentBrakes(0.0)\n";
    emulator.get_input( 7 ).init( 0.0 );
    emulator.call_input( 7 );
    Log::debug << "set_trajectory_length(5)\n";
    emulator.get_input( 8 ).init( 5 );
    emulator.call_input( 8 );
    double x[6] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
    double y[6] = { 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 };
    Log::debug << "set_trajectory_x({0.01, 0.02, 0.03, 0.04, 0.05, 0.06}, 6)\n";
    emulator.get_input( 9 ).init( 6, x );
    emulator.call_input( 9 );
    Log::debug << "set_trajectory_y({ 0.01, 0.01, 0.02, 0.02, 0.01, 0.01 }, 6)\n";
    emulator.get_input( 10 ).init( 6, y );
    emulator.call_input( 10 );
    
    Log::debug << "exec()\n";
    emulator.simulation_time = 1000000000L;
    emulator.call_void( emulator.exec_address );
    
    /*
        { "engine", VALUE_TYPE::DOUBLE },   //0
        { "steering", VALUE_TYPE::DOUBLE }, //1
        { "brakes", VALUE_TYPE::DOUBLE }    //2
    */
    Log::debug << "jni_get_engine()=";
    emulator.call_output( 0 );
    double engine = emulator.get_output( 0 ).double_value;
    Log::debug << to_string( engine ) << "\n";
    Log::debug << "jni_get_steering()=";
    emulator.call_output( 1 );
    double steering = emulator.get_output( 1 ).double_value;
    Log::debug << to_string( steering ) << "\n";
    Log::debug << "jni_get_brakes()=";
    emulator.call_output( 2 );
    double brakes = emulator.get_output( 2 ).double_value;
    Log::debug << to_string( brakes ) << "\n";
    
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
    
    loaded = true;
    return true;
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
