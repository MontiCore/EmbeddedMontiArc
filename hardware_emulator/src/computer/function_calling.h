#pragma once
#include "computer/registers.h"
#include "computer/memory.h"

struct FunctionCalling {

    virtual void set_params( ulong p1 ) = 0;
    virtual void set_params( ulong p1, ulong p2 ) = 0;
    virtual void set_params( ulong p1, ulong p2, ulong p3 ) = 0;
    virtual void set_params( ulong p1, ulong p2, ulong p3, ulong p4 ) = 0;
    
    virtual ulong get_param1() = 0;
    virtual ulong get_param2() = 0;
    virtual ulong get_param3() = 0;
    virtual ulong get_param4() = 0;
    
    virtual ulong get_return() = 0;
    virtual void set_return( ulong r ) = 0;
    virtual ~FunctionCalling() {}
};









struct StackCall {
    Registers &registers;
    VirtualStack &stack;
    
    ///Add from last to first
    void push_param_64( ulong p );
    void push_param_32( uint p );
    
    ulong pop_param_64();
    uint push_param_32();
    
    ulong get_return();
    void set_return( ulong r );
    
    StackCall( Registers &registers, VirtualStack &stack ) : registers( registers ), stack( stack ) {}
};