#include "dll_interface.h"
#include "utility.h"

using namespace std;


void LOADED_DLL::DllInterface::test_main() {

    computer.init();
    if ( !computer.loaded() )
        return;
    os_windows.init( computer );
    if ( !os_windows.load_dll( "loaded_dll.dll" ) )
        return;
        
    init();
    Utility::color_def();
    cout << "test_method()" << endl;
    test_method();
    
}

void LOADED_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[TEST_METHOD] = computer.sys_calls.get_syscall( "loaded_dll.dll", "test_method" );
}


void LOADED_DLL::DllInterface::test_method() {
    computer.call( addresses[TEST_METHOD] );
}


bool ADD_DLL::DllInterface::test_main() {

    computer.init();
    if ( !computer.loaded() )
        return false;
    os_windows.init( computer );
    if ( !os_windows.load_dll( "SampleDLL.dll" ) )
        return false;
        
    init();
    Utility::color_def();
    cout << "add(2,3):" << endl;
    auto res = add( 2, 3 );
    cout << "Result=" << res << endl;
    
    return true;
}

void ADD_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[ADD] = computer.sys_calls.get_syscall( "SampleDLL.dll", "?add@@YAHHH@Z" );
}

int ADD_DLL::DllInterface::add( int a, int b ) {
    computer.fast_call.arg2.set_params( *( ( uint32_t * )&a ), *( ( uint32_t * )&b ) );
    computer.call( addresses[ADD] );
    auto res = computer.fast_call.get_return();
    return *( ( int * ) & ( res ) );
}


void AUTOPILOT_DLL::DllInterface::test_main() {

    computer.init();
    if ( !computer.loaded() )
        return;
    os_windows.init( computer );
    if ( !os_windows.load_dll( "AutopilotModel.dll" ) )
        return;
        
    init();
    Utility::color_def();
    cout << endl << endl << "init(0,0)" << endl << endl;
    init( 0, 0 );
    
}

void AUTOPILOT_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[INIT] = computer.sys_calls.get_syscall( "AutopilotModel.dll",
                      "Java_simulator_integration_AutopilotAdapter_init" );
}


void AUTOPILOT_DLL::DllInterface::init( void *a, void *b ) {
    computer.fast_call.arg2.set_params( *( ( uint64_t * )&a ), *( ( uint64_t * )&b ) );
    computer.call( addresses[INIT] );
}
