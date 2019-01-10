#pragma once
#include "computer/registers.h"

/*
    A Simple wrapper for the Fast Call standard, putting the parameters of a function call in the
    correct registers.
    Also give the return value from function calls.
    All the parameters and return values are 64bit unsigned longs, which are read/copied as-is
    in the registers.
*/
struct FastCall {
    Registers &registers;
    
    void set_params( ulong p1 );
    void set_params( ulong p1, ulong p2 );
    void set_params( ulong p1, ulong p2, ulong p3 );
    void set_params( ulong p1, ulong p2, ulong p3, ulong p4 );
    
    ulong get_param1();
    ulong get_param2();
    ulong get_param3();
    ulong get_param4();
    
    ulong get_return();
    void set_return( ulong r );
    
    FastCall( Registers &registers ) : registers( registers ) {}
};