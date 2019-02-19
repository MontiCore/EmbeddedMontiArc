#include "dll_interface.h"
#include "utility.h"
#include "os_windows/windows_calls.h"

using namespace std;


void LOADED_DLL::DllInterface::init() {
    addresses.init( FUNCTION_COUNT );
    addresses[TEST_METHOD] = computer.sys_calls.get_syscall( "loaded_dll.dll", "test_method" );
}


void LOADED_DLL::DllInterface::test_method() {
    call_success = computer.call( addresses[TEST_METHOD] );
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

