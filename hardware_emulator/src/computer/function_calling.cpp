/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "function_calling.h"





void StackCall::push_param_64( ulong p ) {
    stack.push_long( p );
}

void StackCall::push_param_32( uint p ) {
    stack.push_int( p );
}

ulong StackCall::pop_param_64() {
    return stack.pop_long();
}

uint StackCall::push_param_32() {
    return stack.pop_int();
}

ulong StackCall::get_return() {
    return registers.get_rax();
}

void StackCall::set_return( ulong r ) {
    registers.set_rax( r );
}
