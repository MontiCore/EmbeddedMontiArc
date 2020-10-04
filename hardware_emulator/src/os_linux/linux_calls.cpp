/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "linux_calls.h"
#include <cmath>

using namespace std;

void LinuxCalls::add_linux_calls( SystemCalls &sys_calls ) {
    std::string mod = "SYSTEM";
    const char *reason = "linux";
    sys_calls.add_syscall( SysCall( "malloc", mod, malloc ), reason );
    sys_calls.add_syscall( SysCall( "_Znwm", mod, operatornew ), reason );
    sys_calls.add_syscall( SysCall( "_Znam", mod, operatornew ), reason );
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
    sys_calls.add_syscall( SysCall( "memset", mod, memset ), reason );
    sys_calls.add_syscall( SysCall( "strtoll", mod, strtoll ), reason );
    sys_calls.add_syscall( SysCall( "strtod", mod, strtod ), reason );
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

bool LinuxCalls::operatornew( Computer &computer ) {
    auto byte_count = computer.func_call->get_param1_64();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call->set_return_64( addr );
    else{
        computer.func_call->set_return_64( 0 );
        std::cerr << "Warning: no more memory for operator new" << std::endl;
    }
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


bool LinuxCalls::memset( Computer &computer ) {
    // void * memset ( void * ptr, int value, size_t num );
    auto ptr = computer.func_call->get_param1_64();
    auto value = computer.func_call->get_param2_64();
    auto num = computer.func_call->get_param3_64();
    computer.func_call->set_return_64(ptr);
    if (ptr == 0) return true;

    for (auto i : urange(num/8)){
        computer.memory.write_long_word(ptr+8*i, value);
    }
    for (auto i : urange(num%8)){
        computer.memory.write_memory(ptr+num/8+i, 1, &value);
    }

    return true;
}


bool LinuxCalls::strtoll( Computer &computer ) {
    // long long int strtoll (const char* str, char** endptr, int base);
    auto str = computer.func_call->get_param1_64();
    auto endptr = computer.func_call->get_param2_64();
    auto base = computer.func_call->get_param3_64();
    if (str == 0){
        computer.func_call->set_return_64(0);
        return true;
    }
    auto str_read = computer.memory.read_str(str);
    char *ep;
    slong res = ::strtoll(str_read, &ep, base);
    if (endptr != 0){
        computer.memory.write_long_word(endptr, str+ep-str_read);
    }

    computer.func_call->set_return_64(res);
    return true;
}
bool LinuxCalls::strtod( Computer &computer ) {
    // double strtod (const char* str, char** endptr);
    auto str = computer.func_call->get_param1_64();
    auto endptr = computer.func_call->get_param2_64();
    if (str == 0){
        computer.func_call->set_return_double(std::nan(nullptr));
        return true;
    }
    auto str_read = computer.memory.read_str(str);
    char *ep;
    double res = ::strtod(str_read, &ep);
    if (endptr != 0){
        computer.memory.write_long_word(endptr, str+ep-str_read);
    }

    computer.func_call->set_return_double(res);
    return true;
}
