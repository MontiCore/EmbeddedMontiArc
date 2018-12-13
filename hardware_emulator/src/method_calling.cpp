#include "method_calling.h"



ulong FastCall::FastCall1::get_param1() {
    return registers.get_rdx();
}

void FastCall::FastCall1::set_param( ulong p1 ) {
    registers.set_rdx( p1 );
}

ulong FastCall::FastCall2::get_param1() {
    return registers.get_rcx();
}

ulong FastCall::FastCall2::get_param2() {
    return registers.get_rdx();
}

void FastCall::FastCall2::set_params( ulong p1, ulong p2 ) {
    registers.set_rcx( p1 );
    registers.set_rdx( p2 );
}

ulong FastCall::FastCall3::get_param1() {
    return registers.get_rcx();
}

ulong FastCall::FastCall3::get_param2() {
    return registers.get_rdx();
}

ulong FastCall::FastCall3::get_param3() {
    return registers.get_r8d();
}

void FastCall::FastCall3::set_params( ulong p1, ulong p2, ulong p3 ) {
    registers.set_rcx( p1 );
    registers.set_rdx( p2 );
    registers.set_r8d( p3 );
}

ulong FastCall::get_return() {
    return registers.get_rax();
}
void FastCall::set_return( ulong r ) {
    registers.set_rax( r );
}