#include "dll_interface.h"
#include "utility.h"
#include "os_windows/windows_calls.h"

using namespace std;


bool LOADED_DLL::DllInterface::test_main() {
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    
    computer.init();
    if ( !computer.loaded() )
        return false;
    os_windows.init( computer );
    
    
    WindowsCalls calls;
    calls.add_windows_calls( computer.sys_calls, os_windows );
    if ( !os_windows.load_dll( "loaded_dll.dll" ) )
        return false;
        
        
    init();
    Log::debug << "test_method()\n";
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
    computer.debug.debug = true;
    computer.debug.d_mem = false;
    computer.debug.d_regs = false;
    computer.debug.d_reg_update = false;
    computer.debug.d_syscalls = false;
    
    computer.init();
    if ( !computer.loaded() )
        return false;
        
        
    os_windows.init( computer );
    WindowsCalls calls;
    calls.add_windows_calls( computer.sys_calls, os_windows );
    if ( !os_windows.load_dll( "SampleDLL.dll" ) )
        return false;
        
        
    init();
    Log::debug << "add(2,3):\n";
    auto res = add( 2, 3 );
    Log::debug << "Result=" << res << "\n";
    
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

