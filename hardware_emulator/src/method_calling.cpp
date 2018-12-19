#include "method_calling.h"



void FastCall::set_params( ulong p1 ) {
    registers.set_rcx( p1 );
}

void FastCall::set_params( ulong p1, ulong p2 ) {
    registers.set_rcx( p1 );
    registers.set_rdx( p2 );
}
void FastCall::set_params( ulong p1, ulong p2, ulong p3 ) {
    registers.set_rcx( p1 );
    registers.set_rdx( p2 );
    registers.set_r8( p3 );
}
void FastCall::set_params( ulong p1, ulong p2, ulong p3, ulong p4 ) {
    registers.set_rcx( p1 );
    registers.set_rdx( p2 );
    registers.set_r8( p3 );
    registers.set_r9( p4 );
}

ulong FastCall::get_param1() {
    return registers.get_rcx();
}

ulong FastCall::get_param2() {
    return registers.get_rdx();
}

ulong FastCall::get_param3() {
    return registers.get_r8();
}
ulong FastCall::get_param4() {
    return registers.get_r9();
}



ulong FastCall::get_return() {
    return registers.get_rax();
}
void FastCall::set_return( ulong r ) {
    registers.set_rax( r );
}