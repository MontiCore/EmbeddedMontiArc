/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "linux_system_calls.h"
#include <cmath>

using namespace std;

void LinuxSystemCalls::add_linux_calls( SystemCalls &sys_calls ) {
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
    sys_calls.add_syscall(SysCall("gettext", mod, [](Computer& c) {
        // char * gettext (const char * msgid);
        // => return msgid
        c.func_call_linux.set_return_64(c.func_call_linux.get_param1_64());
        return true;
    }), reason);

}

bool LinuxSystemCalls::malloc( Computer &computer ) {
    auto byte_count = computer.func_call_linux.get_param1_64();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call_linux.set_return_64( addr );
    else
        computer.func_call_linux.set_return_64( 0 );
    return true;
}

bool LinuxSystemCalls::operatornew( Computer &computer ) {
    auto byte_count = computer.func_call_linux.get_param1_64();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call_linux.set_return_64( addr );
    else{
        computer.func_call_linux.set_return_64( 0 );
        Log::err.log("Warning: no more memory for operator new");
    }
    return true;
}

bool LinuxSystemCalls::time( Computer &computer ) {
    computer.func_call_linux.set_return_64( 0x10203040 );
    return true;
}

bool LinuxSystemCalls::srand( Computer &computer ) {
    auto t = computer.func_call_linux.get_param1_64();
    ::srand( ( uint )t );
    return true;
}

bool LinuxSystemCalls::posix_memalign( Computer &computer ) {
    auto target_pointer = computer.func_call_linux.get_param1_64();
    auto alignment = computer.func_call_linux.get_param2_64();
    auto size = computer.func_call_linux.get_param3_64();
    if ( computer.debug.syscalls() )
        Log::sys.log("posix_memalign(%s, %llu, %llu)", to_hex(target_pointer).c_str(), alignment, size);
    ulong addr;
    if ( computer.heap.alloc( size + alignment, addr ) ) {
        computer.func_call_linux.set_return_64( 0 );
        computer.memory.write_long_word( target_pointer, ( ( ( addr - 1 ) / alignment ) + 1 )*alignment );
    }
    else
        computer.func_call_linux.set_return_64( 1 );
        
    return true;
}

bool LinuxSystemCalls::sqrt( Computer &computer ) {
    double v = computer.func_call_linux.get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("sqrt(%LF)", v);
    computer.func_call_linux.set_return_double( ::sqrt( v ) );
    return true;
}

bool LinuxSystemCalls::sin( Computer &computer ) {
    double v = computer.func_call_linux.get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("sin(%LF)", v);
    computer.func_call_linux.set_return_double( ::sin( v ) );
    return true;
}

bool LinuxSystemCalls::cos( Computer &computer ) {
    double v = computer.func_call_linux.get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("cos(%LF)", v);
    computer.func_call_linux.set_return_double( ::cos( v ) );
    return true;
}

bool LinuxSystemCalls::acos( Computer &computer ) {
    double v = computer.func_call_linux.get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("acos(%LF)", v);
    computer.func_call_linux.set_return_double( ::acos( v ) );
    return true;
}

bool LinuxSystemCalls::exp( Computer &computer ) {
    double v = computer.func_call_linux.get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("exp(%LF)", v);
    computer.func_call_linux.set_return_double( ::exp( v ) );
    return true;
}

bool LinuxSystemCalls::log( Computer &computer ) {
    double v = computer.func_call_linux.get_param1_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("log(%LF)", v);
    computer.func_call_linux.set_return_double( ::log( v ) );
    return true;
}

bool LinuxSystemCalls::atan2( Computer &computer ) {
    double v1 = computer.func_call_linux.get_param1_double();
    double v2 = computer.func_call_linux.get_param2_double();
    if ( computer.debug.syscalls() )
        Log::sys.log("atan2(%LF, %LF)", v1, v2);
    computer.func_call_linux.set_return_double( ::atan2( v1, v2 ) );
    return true;
}

bool LinuxSystemCalls::memcpy( Computer &computer ) {
    auto target_pointer = computer.func_call_linux.get_param1_64();
    auto source_pointer = computer.func_call_linux.get_param2_64();
    auto size = computer.func_call_linux.get_param3_64();
    if ( computer.debug.syscalls() )
        Log::sys.log("memcpy(%s, %s, %llu)", to_hex(target_pointer).c_str(), to_hex(source_pointer).c_str(), size);
    auto r = computer.memory.read_memory( source_pointer, size );
    computer.memory.write_memory( target_pointer, size, r );
    return true;
}


bool LinuxSystemCalls::memset( Computer &computer ) {
    // void * memset ( void * ptr, int value, size_t num );
    auto ptr = computer.func_call_linux.get_param1_64();
    auto value = computer.func_call_linux.get_param2_64();
    auto num = computer.func_call_linux.get_param3_64();
    computer.func_call_linux.set_return_64(ptr);
    if (ptr == 0) return true;

    for (auto i : urange(num/8)){
        computer.memory.write_long_word(ptr+8*i, value);
    }
    for (auto i : urange(num%8)){
        computer.memory.write_memory(ptr+num/8+i, 1, &value);
    }

    return true;
}


bool LinuxSystemCalls::strtoll( Computer &computer ) {
    // long long int strtoll (const char* str, char** endptr, int base);
    auto str = computer.func_call_linux.get_param1_64();
    auto endptr = computer.func_call_linux.get_param2_64();
    auto base = computer.func_call_linux.get_param3_64();
    if (str == 0){
        computer.func_call_linux.set_return_64(0);
        return true;
    }
    auto str_read = computer.memory.read_str(str);
    char *ep;
    slong res = ::strtoll(str_read, &ep, base);
    if (endptr != 0){
        computer.memory.write_long_word(endptr, str+ep-str_read);
    }

    computer.func_call_linux.set_return_64(res);
    return true;
}
bool LinuxSystemCalls::strtod( Computer &computer ) {
    // double strtod (const char* str, char** endptr);
    auto str = computer.func_call_linux.get_param1_64();
    auto endptr = computer.func_call_linux.get_param2_64();
    if (str == 0){
        computer.func_call_linux.set_return_double(std::numeric_limits<double>::quiet_NaN());
        return true;
    }
    auto str_read = computer.memory.read_str(str);
    char *ep;
    double res = ::strtod(str_read, &ep);
    if (endptr != 0){
        computer.memory.write_long_word(endptr, str+ep-str_read);
    }

    computer.func_call_linux.set_return_double(res);
    return true;
}
