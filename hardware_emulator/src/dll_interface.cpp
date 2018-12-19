#include "dll_interface.h"
#include "utility.h"
#include "os_windows/windows_calls.h"

using namespace std;


bool LOADED_DLL::DllInterface::test_main() {

    computer.init();
    if ( !computer.loaded() )
        return false;
    os_windows.init( computer );
    
    computer.debug.debug = false;
    
    WindowsCalls calls;
    calls.add_windows_calls( computer.sys_calls, os_windows );
    if ( !os_windows.load_dll( "loaded_dll.dll" ) )
        return false;
        
        
    init();
    Utility::color_def();
    cout << "test_method()" << endl;
    test_method();
    
    return call_success;
}

void LOADED_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[TEST_METHOD] = computer.sys_calls.get_syscall( "loaded_dll.dll", "test_method" );
}


void LOADED_DLL::DllInterface::test_method() {
    call_success = computer.call( addresses[TEST_METHOD] );
}


bool ADD_DLL::DllInterface::test_main() {

    computer.init();
    if ( !computer.loaded() )
        return false;
        
    computer.debug.debug = false;
    
    os_windows.init( computer );
    WindowsCalls calls;
    calls.add_windows_calls( computer.sys_calls, os_windows );
    if ( !os_windows.load_dll( "SampleDLL.dll" ) )
        return false;
        
        
    init();
    Utility::color_def();
    cout << "add(2,3):" << endl;
    auto res = add( 2, 3 );
    cout << "Result=" << res << endl;
    
    return call_success;
}

void ADD_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[ADD] = computer.sys_calls.get_syscall( "SampleDLL.dll", "?add@@YAHHH@Z" );
}

int ADD_DLL::DllInterface::add( int a, int b ) {
    computer.fast_call.set_params( *( ( uint32_t * )&a ), *( ( uint32_t * )&b ) );
    call_success = computer.call( addresses[ADD] );
    auto res = computer.fast_call.get_return();
    return *( ( int * ) & ( res ) );
}


bool AUTOPILOT_DLL::DllInterface::test_main() {

    computer.init();
    if ( !computer.loaded() )
        return false;
        
    //computer.debug.debug = false;
    
    os_windows.init( computer );
    WindowsCalls calls;
    calls.add_windows_calls( computer.sys_calls, os_windows );
    if ( !os_windows.load_dll( "AutopilotModel.dll" ) )
        return false;
        
        
    init();
    Utility::color_def();
    cout << endl << endl << "init(0,0)" << endl << endl;
    init( 0, 0 );
    
    return call_success;
}

void AUTOPILOT_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[INIT] = computer.sys_calls.get_syscall( "AutopilotAdapter.dll",
                      "Java_simulator_integration_AutopilotAdapter_init" );
    /*addresses[INIT] = computer.sys_calls.get_syscall( "AutopilotModel.dll",
                      "Java_simulator_integration_AutopilotAdapter_init" );*/
}


void AUTOPILOT_DLL::DllInterface::init( void *a, void *b ) {
    computer.fast_call.set_params( *( ( uint64_t * )&a ), *( ( uint64_t * )&b ) );
    call_success = computer.call( addresses[INIT] );
}
