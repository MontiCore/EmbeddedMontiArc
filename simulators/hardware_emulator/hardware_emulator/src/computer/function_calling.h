/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "computer/registers.h"
#include "computer/memory.h"

/*
    This interface must be implemented and registered in the computer by the OS emulations.
    This interface abstracts how arguments are passed to functions and how the return value is read.

    //A possible modification could be to have a non interface FunctionCalling object that has an
    arrays containing the UC_X86_REG_* values of the register corresponding to a data type and argument position.
    The OSs would then just have to fill the arrays with the correct register ids.
    Ex:
    uc_x86_reg integer_registers[] = { UC_X86_REG_RDI, UC_X86_REG_RSI, UC_X86_REG_RDX, UC_X86_REG_RCX }
    for Linux
    uc_x86_reg integer_registers[] = { UC_X86_REG_RCX, UC_X86_REG_RDX, UC_X86_REG_R8, UC_X86_REG_R9 }
    for Windows

    and
    uc_x86_reg floating_point_registers[] = { UC_X86_REG_XMM0, UC_X86_REG_XMM1, UC_X86_REG_XMM2, UC_X86_REG_XMM3 }
    uc_x86_reg floating_point_return_register = UC_X86_REG_XMM0;
    uc_x86_reg integer_return_register = UC_X86_REG_RAX;
    for both Linux and Windows

    This interface might have to be updated/changed if new arguments appear in autopilots that are not passed by registers
    but on the stack.
*/
struct FunctionCalling {
    //Caller
    //virtual void set_params_64( ulong p1 ) = 0;
    //virtual void set_params_64( ulong p1, ulong p2 ) = 0;
    //virtual void set_params_64( ulong p1, ulong p2, ulong p3 ) = 0;
    //virtual void set_params_64( ulong p1, ulong p2, ulong p3, ulong p4 ) = 0;
    //
    //virtual void set_params_32( uint p1 ) = 0;
    //virtual void set_params_32( uint p1, uint p2 ) = 0;
    //virtual void set_params_32( uint p1, uint p2, uint p3 ) = 0;
    //virtual void set_params_32( uint p1, uint p2, uint p3, uint p4 ) = 0;
    //
    //virtual void set_params_double( double p1 ) = 0;
    //virtual void set_params_double( double p1, double p2 ) = 0;
    //virtual void set_params_double( double p1, double p2, double p3 ) = 0;
    //virtual void set_params_double( double p1, double p2, double p3, double p4 ) = 0;
    //
    //virtual void set_params_float( float p1 ) = 0;
    //virtual void set_params_float( float p1, float p2 ) = 0;
    //virtual void set_params_float( float p1, float p2, float p3 ) = 0;
    //virtual void set_params_float( float p1, float p2, float p3, float p4 ) = 0;
    //
    //virtual void set_param1_64( ulong p ) = 0;
    //virtual void set_param2_64( ulong p ) = 0;
    //virtual void set_param3_64( ulong p ) = 0;
    //virtual void set_param4_64( ulong p ) = 0;
    //virtual void set_param5_64(ulong p) = 0;
    //virtual void set_param6_64(ulong p) = 0;
    //
    //virtual void set_param1_32( uint p ) = 0;
    //virtual void set_param2_32( uint p ) = 0;
    //virtual void set_param3_32( uint p ) = 0;
    //virtual void set_param4_32( uint p ) = 0;
    //virtual void set_param5_32(uint p) = 0;
    //virtual void set_param6_32(uint p) = 0;
    //
    //virtual void set_param1_double( double p ) = 0;
    //virtual void set_param2_double( double p ) = 0;
    //virtual void set_param3_double( double p ) = 0;
    //virtual void set_param4_double( double p ) = 0;
    //
    //virtual void set_param1_float( float p ) = 0;
    //virtual void set_param2_float( float p ) = 0;
    //virtual void set_param3_float( float p ) = 0;
    //virtual void set_param4_float( float p ) = 0;

    //// template<typename T> void set_param1(T v) {}
    //// template<typename T> void set_param2(T v) {}
    //// template<typename T> void set_param3(T v) {}
    //// template<typename T> void set_param4(T v) {}
    //inline void set_param1(double v) { set_param1_double(v); }
    //inline void set_param2(double v) { set_param2_double(v); }
    //inline void set_param3(double v) { set_param3_double(v); }
    //inline void set_param4(double v) { set_param4_double(v); }
    //inline void set_param1(float v) { set_param1_float(v); }
    //inline void set_param2(float v) { set_param2_float(v); }
    //inline void set_param3(float v) { set_param3_float(v); }
    //inline void set_param4(float v) { set_param4_float(v); }
    //inline void set_param1(int v) { set_param1_32(v); }
    //inline void set_param2(int v) { set_param1_32(v); }
    //inline void set_param3(int v) { set_param1_32(v); }
    //inline void set_param4(int v) { set_param1_32(v); }
    //inline void set_param1(unsigned char v) { set_param1_32(v); }
    //inline void set_param2(unsigned char v) { set_param1_32(v); }
    //inline void set_param3(unsigned char v) { set_param1_32(v); }
    //inline void set_param4(unsigned char v) { set_param1_32(v); }
    //inline void set_param1(bool v) { set_param1_32(v); }
    //inline void set_param2(bool v) { set_param1_32(v); }
    //inline void set_param3(bool v) { set_param1_32(v); }
    //inline void set_param4(bool v) { set_param1_32(v); }
    //
    //virtual ulong get_return_64() = 0;
    //virtual uint get_return_32() = 0;
    //virtual double get_return_double() = 0;
    //virtual float get_return_float() = 0;
    //virtual char get_return_char() = 0;


    //template<typename T> T get_return() {}
    //inline double get_return_f64() { return get_return_double(); }
    //inline float get_return_f32() { return get_return_float(); }
    //inline int get_return_i32() { return get_return_32(); }
    //inline unsigned char get_return_i8() { return get_return_32(); }
    //inline bool get_return_bool() { return get_return_32(); }
    //
    //
    ////Callee
    //virtual ulong get_param1_64() = 0;
    //virtual ulong get_param2_64() = 0;
    //virtual ulong get_param3_64() = 0;
    //virtual ulong get_param4_64() = 0;
    //
    //virtual uint get_param1_32() = 0;
    //virtual uint get_param2_32() = 0;
    //virtual uint get_param3_32() = 0;
    //virtual uint get_param4_32() = 0;
    //
    //virtual double get_param1_double() = 0;
    //virtual double get_param2_double() = 0;
    //virtual double get_param3_double() = 0;
    //virtual double get_param4_double() = 0;
    //
    //virtual float get_param1_float() = 0;
    //virtual float get_param2_float() = 0;
    //virtual float get_param3_float() = 0;
    //virtual float get_param4_float() = 0;
    //
    //virtual void set_return_64( ulong r ) = 0;
    //virtual void set_return_32( uint r ) = 0;
    //virtual void set_return_double( double r ) = 0;
    
    
    virtual ~FunctionCalling() {}
};








/*
    This might be used by arguments that must be passed on the stack rather than through registers
*/
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
