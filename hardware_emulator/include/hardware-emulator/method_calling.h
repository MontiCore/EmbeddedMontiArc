#pragma once
#include "computer/registers.h"

struct FastCall {
    struct FastCall1 {
        Registers &registers;
        ulong get_param1();
        void set_param( ulong p1 );
        FastCall1( Registers &registers ) : registers( registers ) {}
    };
    struct FastCall2 {
        Registers &registers;
        ulong get_param1();
        ulong get_param2();
        void set_params( ulong p1, ulong p2 );
        FastCall2( Registers &registers ) : registers( registers ) {}
    };
    struct FastCall3 {
        Registers &registers;
        ulong get_param1();
        ulong get_param2();
        ulong get_param3();
        void set_params( ulong p1, ulong p2, ulong p3 );
        FastCall3( Registers &registers ) : registers( registers ) {}
    };
    
    Registers &registers;
    FastCall1 arg1;
    FastCall2 arg2;
    FastCall3 arg3;
    
    ulong get_return();
    void set_return( ulong r );
    
    FastCall( Registers &registers ) : registers( registers ), arg1( registers ), arg2( registers ),
        arg3( registers ) {}
};