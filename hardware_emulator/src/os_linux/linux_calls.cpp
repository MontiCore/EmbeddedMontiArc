#include "linux_calls.h"

using namespace std;

void LinuxCalls::add_linux_calls( SystemCalls &sys_calls ) {
    std::string mod = "SYSTEM";
    sys_calls.add_syscall( SysCall( "malloc", mod, malloc ) );
}

bool LinuxCalls::malloc( Computer &inter, SysCall &syscall ) {
    auto byte_count = inter.func_call->get_param1();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( inter.heap.alloc( byte_count, addr ) )
        inter.func_call->set_return( addr );
    else
        inter.func_call->set_return( 0 );
    return true;
}
