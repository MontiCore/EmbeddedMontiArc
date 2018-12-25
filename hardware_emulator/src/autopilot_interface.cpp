#include "autopilot_interface.h"
#include "utility.h"
#include "os_windows/windows_calls.h"


using namespace std;

AutopilotInterface AutopilotInterface::instance;

bool AutopilotInterface::test_main() {
    if ( !init() )
        return false;
    Utility::color_def();
    cout << "init(0,0)" << endl;
    jni_init();
    
    
    
    
    
    
    return call_success;
}
enum Functions {
    SET_TIME_INC,
    SET_VELOCITY,
    SET_X,
    SET_Y,
    SET_COMPASS,
    SET_ENGINE,
    SET_STEERING,
    SET_BRAKES,
    SET_TRAJECTORY_LENGTH,
    SET_TRAJECTORY_X,
    SET_TRAJECTORY_Y,
    GET_ENGINE,
    GET_STEERING,
    GET_BRAKES,
    EXEC,
    INIT,
    MESSAGE,
    
    FUNCTION_COUNT
};
bool AutopilotInterface::init() {
    cout << "AutopilotInterface::init()" << endl;
    computer.debug.debug = false;
    computer.init();
    if ( !computer.loaded() )
        return false;
        
        
        
    os_windows.init( computer );
    WindowsCalls calls;
    calls.add_windows_calls( computer.sys_calls, os_windows );
    cout << "Load DLL AutopilotModel.dll" << endl;
    if ( !os_windows.load_dll( "AutopilotModel.dll" ) )
        return false;
        
    jni_emu.init( computer );
    
    engine_value = 0;
    steering_value = 0;
    brake_value = 0;
    
    simulation_time = 0;
    addresses.init( FUNCTION_COUNT );
    addresses[SET_TIME_INC] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                              "Java_simulator_integration_AutopilotAdapter_set_1timeIncrement" );
    addresses[SET_VELOCITY] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                              "Java_simulator_integration_AutopilotAdapter_set_1currentVelocity" );
    addresses[SET_X] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                       "Java_simulator_integration_AutopilotAdapter_set_1x" );
    addresses[SET_Y] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                       "Java_simulator_integration_AutopilotAdapter_set_1y" );
    addresses[SET_COMPASS] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                             "Java_simulator_integration_AutopilotAdapter_set_1compass" );
    addresses[SET_ENGINE] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                            "Java_simulator_integration_AutopilotAdapter_set_1currentEngine" );
    addresses[SET_STEERING] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                              "Java_simulator_integration_AutopilotAdapter_set_1currentSteering" );
    addresses[SET_BRAKES] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                            "Java_simulator_integration_AutopilotAdapter_set_1currentBrakes" );
    addresses[SET_TRAJECTORY_LENGTH] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                                       "Java_simulator_integration_AutopilotAdapter_set_1trajectory_1length" );
    addresses[SET_TRAJECTORY_X] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                                  "Java_simulator_integration_AutopilotAdapter_set_1trajectory_1x" );
    addresses[SET_TRAJECTORY_Y] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                                  "Java_simulator_integration_AutopilotAdapter_set_1trajectory_1y" );
    addresses[GET_ENGINE] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                            "Java_simulator_integration_AutopilotAdapter_get_1engine" );
    addresses[GET_STEERING] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                              "Java_simulator_integration_AutopilotAdapter_get_1steering" );
    addresses[GET_BRAKES] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                            "Java_simulator_integration_AutopilotAdapter_get_1brakes" );
    addresses[EXEC] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                      "Java_simulator_integration_AutopilotAdapter_exec" );
    addresses[INIT] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                      "Java_simulator_integration_AutopilotAdapter_init" );
    addresses[MESSAGE] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                         "Java_simulator_integration_AutopilotAdapter_message" );
                         
    loaded = true;
    return true;
}

void AutopilotInterface::jni_exec() {
    call_success = jni_emu.call( addresses[EXEC] );
}


void AutopilotInterface::jni_init() {
    call_success = jni_emu.call( addresses[INIT] );
}

char *AutopilotInterface::jni_message( char *msg ) {
    return nullptr;
}


void AutopilotInterface::jni_set_x( double x ) {
    call_success = jni_emu.call( addresses[SET_X], x );
}

void AutopilotInterface::jni_set_y( double y ) {
    call_success = jni_emu.call( addresses[SET_Y], y );
}

void AutopilotInterface::jni_set_compass( double compass ) {
    call_success = jni_emu.call( addresses[SET_COMPASS], compass );
}

void AutopilotInterface::jni_set_currentEngine( double currentEngine ) {
    call_success = jni_emu.call( addresses[SET_ENGINE], currentEngine );
}

void AutopilotInterface::jni_set_currentSteering( double currentSteering ) {
    call_success = jni_emu.call( addresses[SET_STEERING], currentSteering );
}

void AutopilotInterface::jni_set_currentBrakes( double currentBrakes ) {
    call_success = jni_emu.call( addresses[SET_BRAKES], currentBrakes );
}

void AutopilotInterface::jni_set_trajectory_length( int length ) {
    call_success = jni_emu.call( addresses[SET_TRAJECTORY_LENGTH], ( ulong )length );
}

void AutopilotInterface::jni_set_trajectory_x( double *x, uint count ) {
    call_success = jni_emu.call( addresses[SET_TRAJECTORY_X], x, count );
}

void AutopilotInterface::jni_set_trajectory_y( double *y, uint count ) {
    call_success = jni_emu.call( addresses[SET_TRAJECTORY_Y], y, count );
}

double AutopilotInterface::jni_get_engine() {
    if ( !running() ) {
        call_success = jni_emu.call( addresses[GET_ENGINE] );
        engine_value = jni_emu.return_double();
    }
    return engine_value;
}

double AutopilotInterface::jni_get_steering() {
    if ( !running() ) {
        call_success = jni_emu.call( addresses[GET_STEERING] );
        steering_value = jni_emu.return_double();
    }
    return steering_value;
}

double AutopilotInterface::jni_get_brakes() {
    if ( !running() ) {
        call_success = jni_emu.call( addresses[GET_BRAKES] );
        brake_value = jni_emu.return_double();
    }
    return brake_value;
}

void AutopilotInterface::jni_set_timeIncrement( double timeIncrement ) {
    if ( !running() )
        call_success = jni_emu.call( addresses[SET_TIME_INC], timeIncrement );
}

void AutopilotInterface::jni_set_currentVelocity( double currentVelocity ) {
    if ( !running() )
        call_success = jni_emu.call( addresses[SET_VELOCITY], currentVelocity );
}


