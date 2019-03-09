#pragma once
#include "computer/registers.h"
#include "computer/memory.h"

struct FunctionCalling {
    //Caller
    virtual void set_params_64( ulong p1 ) = 0;
    virtual void set_params_64( ulong p1, ulong p2 ) = 0;
    virtual void set_params_64( ulong p1, ulong p2, ulong p3 ) = 0;
    virtual void set_params_64( ulong p1, ulong p2, ulong p3, ulong p4 ) = 0;
    
    virtual void set_params_32( uint p1 ) = 0;
    virtual void set_params_32( uint p1, uint p2 ) = 0;
    virtual void set_params_32( uint p1, uint p2, uint p3 ) = 0;
    virtual void set_params_32( uint p1, uint p2, uint p3, uint p4 ) = 0;
    
    virtual void set_param1_64( ulong p ) = 0;
    virtual void set_param2_64( ulong p ) = 0;
    virtual void set_param3_64( ulong p ) = 0;
    virtual void set_param4_64( ulong p ) = 0;
    
    virtual void set_param1_32( uint p ) = 0;
    virtual void set_param2_32( uint p ) = 0;
    virtual void set_param3_32( uint p ) = 0;
    virtual void set_param4_32( uint p ) = 0;
    
    virtual void set_param1_double( double p ) = 0;
    virtual void set_param2_double( double p ) = 0;
    virtual void set_param3_double( double p ) = 0;
    virtual void set_param4_double( double p ) = 0;
    
    virtual ulong get_return_64() = 0;
    virtual uint get_return_32() = 0;
    virtual double get_return_double() = 0;
    
    
    //Callee
    virtual ulong get_param1_64() = 0;
    virtual ulong get_param2_64() = 0;
    virtual ulong get_param3_64() = 0;
    virtual ulong get_param4_64() = 0;
    
    virtual uint get_param1_32() = 0;
    virtual uint get_param2_32() = 0;
    virtual uint get_param3_32() = 0;
    virtual uint get_param4_32() = 0;
    
    virtual double get_param1_double() = 0;
    virtual double get_param2_double() = 0;
    virtual double get_param3_double() = 0;
    virtual double get_param4_double() = 0;
    
    virtual void set_return_64( ulong r ) = 0;
    virtual void set_return_32( uint r ) = 0;
    virtual void set_return_double( double r ) = 0;
    
    
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