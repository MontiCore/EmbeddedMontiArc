/* (c) https://github.com/MontiCore/monticore */
#include "linux_calls.h"
#include <cmath>

using namespace std;

void LinuxCalls::add_linux_calls( SystemCalls &sys_calls ) {
    std::string mod = "SYSTEM";
    const char *reason = "linux";
    sys_calls.add_syscall( SysCall( "malloc", mod, malloc ), reason );
    sys_calls.add_syscall( SysCall( "time", mod, time ), reason );
    sys_calls.add_syscall( SysCall( "srand", mod, srand ), reason );
    sys_calls.add_syscall( SysCall( "posix_memalign", mod, posix_memalign ), reason );
    sys_calls.add_syscall( SysCall( "sqrt", mod, sqrt ), reason );
    sys_calls.add_syscall( SysCall( "sin", mod, sin ), reason );
    sys_calls.add_syscall( SysCall( "cos", mod, cos ), reason );
    sys_calls.add_syscall( SysCall( "acos", mod, acos ), reason );
    sys_calls.add_syscall( SysCall( "exp", mod, exp ), reason );
    sys_calls.add_syscall( SysCall( "log", mod, log ), reason );
    sys_calls.add_syscall( SysCall( "atan2", mod, atan2 ), reason );
    sys_calls.add_syscall( SysCall( "memcpy", mod, memcpy ), reason );
}

bool LinuxCalls::malloc( Computer &computer ) {
    auto byte_count = computer.func_call->get_param1_64();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call->set_return_64( addr );
    else
        computer.func_call->set_return_64( 0 );
    return true;
}

bool LinuxCalls::time( Computer &computer ) {
    computer.func_call->set_return_64( 0x10203040 );
    return true;
}

bool LinuxCalls::srand( Computer &computer ) {
    auto t = computer.func_call->get_param1_64();
    ::srand( ( uint )t );
    return true;
}

bool LinuxCalls::posix_memalign( Computer &computer ) {
    auto target_pointer = computer.func_call->get_param1_64();
    auto alignment = computer.func_call->get_param2_64();
    auto size = computer.func_call->get_param3_64();
    if ( computer.debug.syscalls() )
        Log::sys << "posix_memalign(" << to_hex( target_pointer ) << ", " << alignment << ", " << size << ")\n";
    ulong addr;
    if ( computer.heap.alloc( size + alignment, addr ) ) {
        computer.func_call->set_return_64( 0 );
        computer.memory.write_long_word( target_pointer, ( ( ( addr - 1 ) / alignment ) + 1 )*alignment );
    }
    else
        computer.func_call->set_return_64( 1 );
        
    return true;
}

bool LinuxCalls::sqrt( Computer &computer ) {
    double v = computer.func_call->get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys << "sqrt(" << v << ")\n";
    computer.func_call->set_return_double( ::sqrt( v ) );
    return true;
}

bool LinuxCalls::sin( Computer &computer ) {
    double v = computer.func_call->get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys << "sin(" << v << ")\n";
    computer.func_call->set_return_double( ::sin( v ) );
    return true;
}

bool LinuxCalls::cos( Computer &computer ) {
    double v = computer.func_call->get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys << "cos(" << v << ")\n";
    computer.func_call->set_return_double( ::cos( v ) );
    return true;
}

bool LinuxCalls::acos( Computer &computer ) {
    double v = computer.func_call->get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys << "acos(" << v << ")\n";
    computer.func_call->set_return_double( ::acos( v ) );
    return true;
}

bool LinuxCalls::exp( Computer &computer ) {
    double v = computer.func_call->get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys << "exp(" << v << ")\n";
    computer.func_call->set_return_double( ::exp( v ) );
    return true;
}

bool LinuxCalls::log( Computer &computer ) {
    double v = computer.func_call->get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys << "log(" << v << ")\n";
    computer.func_call->set_return_double( ::log( v ) );
    return true;
}

bool LinuxCalls::atan2( Computer &computer ) {
    double v1 = computer.func_call->get_param1_double();
    double v2 = computer.func_call->get_param2_double();
    if ( computer.debug.syscalls() )
        Log::sys << "atan2(" << v1 << ", " << v2 << ")\n";
    computer.func_call->set_return_double( ::atan2( v1, v2 ) );
    return true;
}

bool LinuxCalls::memcpy( Computer &computer ) {
    auto target_pointer = computer.func_call->get_param1_64();
    auto source_pointer = computer.func_call->get_param2_64();
    auto size = computer.func_call->get_param3_64();
    if ( computer.debug.syscalls() )
        Log::sys << "memcpy(" << to_hex( target_pointer ) << ", " << to_hex( source_pointer ) << ", " << size << ")\n";
    auto r = computer.memory.read_memory( source_pointer, size );
    computer.memory.write_memory( target_pointer, size, r );
    return true;
}
