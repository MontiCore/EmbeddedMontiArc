/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "utility/utility.h"

/*
    A Simple wrapper to access the registers of the Unicorn Engine.

    Contains printing methods for debugging. print_changed_registers() compares the
    registers with their last known value (last print_changed_registers()) and only
    shows the updated registers.
*/
struct Registers {
    static constexpr ulong BUFFER_SIZE = 32;
    void *internal_uc = nullptr;
    uchar mini_buff[16] = {0};
    ulong reg = 0;
    ulong reg2 = 0;
    ulong regs[BUFFER_SIZE] = { 0 };
    ulong regs_old[BUFFER_SIZE] = { 0 };
    
    //All main registers
    static int regs_id[BUFFER_SIZE];
    static const char *regs_names[BUFFER_SIZE];
    
    void init( void *uc );
    
    void print_registers();
    void print_changed_registers();
    
    ulong get_eax();
    ulong get_ebx();
    ulong get_ecx();
    ulong get_edx();
    ulong get_rax();
    ulong get_rbx();
    ulong get_rcx();
    ulong get_rdx();
    
    ulong get_r8();
    ulong get_r9();
    
    double get_xmm0();
    double get_xmm1();
    double get_xmm2();
    double get_xmm3();
    
    float get_xmm0_f();
    float get_xmm1_f();
    float get_xmm2_f();
    float get_xmm3_f();
    
    
    ulong get_rsp();
    ulong get_rsi();
    ulong get_rdi();
    ulong get_rip();
    
    ulong get_gs();
    
    void get_gdtr( void *gdtr );
    
    
    void set_eax( ulong val );
    void set_ebx( ulong val );
    void set_ecx( ulong val );
    void set_edx( ulong val );
    void set_rax( ulong val );
    void set_rbx( ulong val );
    void set_rcx( ulong val );
    void set_rdx( ulong val );
    
    
    void set_r8( ulong val );
    void set_r9( ulong val );
    
    void set_xmm0( double val );
    void set_xmm1( double val );
    void set_xmm2( double val );
    void set_xmm3( double val );
    
    void set_xmm0_f( float val );
    void set_xmm1_f( float val );
    void set_xmm2_f( float val );
    void set_xmm3_f( float val );
    
    void set_rsp( ulong val );
    void set_rsi( ulong val );
    void set_rdi( ulong val );
    void set_rip( ulong val );
    
    void set_gs( ulong val );
    
    void set_gdtr( void *gdtr );
};
