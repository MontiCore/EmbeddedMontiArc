/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "computer/registers.h"
#include <unicorn/unicorn.h>


int Registers::regs_id[Registers::BUFFER_SIZE] = {
    UC_X86_REG_RAX,
    UC_X86_REG_RBX,
    UC_X86_REG_RCX,
    UC_X86_REG_RDX,
    UC_X86_REG_RBP,
    UC_X86_REG_RSP,
    UC_X86_REG_RSI,
    UC_X86_REG_RDI,
    
    UC_X86_REG_R8,
    UC_X86_REG_R9,
    UC_X86_REG_R10,
    UC_X86_REG_R11,
    UC_X86_REG_R12,
    UC_X86_REG_R13,
    UC_X86_REG_R14,
    UC_X86_REG_R15,
    
    UC_X86_REG_RIP,
    UC_X86_REG_EFLAGS,
    
    
    UC_X86_REG_XMM0,
    UC_X86_REG_XMM1,
    UC_X86_REG_XMM2,
    UC_X86_REG_XMM3,
};

const char *Registers::regs_names[Registers::BUFFER_SIZE] = {
    "RAX",
    "RBX",
    "RCX",
    "RDX",
    "RBP",
    "RSP",
    "RSI",
    "RDI",
    
    "R8",
    "R9",
    "R10",
    "R11",
    "R12",
    "R13",
    "R14",
    "R15",
    
    "RIP",
    "EFLAGS",
    
    "XMM0",
    "XMM1",
    "XMM2",
    "XMM3",
};


void Registers::init( void *uc ) {
    this->internal_uc = uc;
    for ( auto i : urange( BUFFER_SIZE ) )
        regs_old[( uint )i] = 0;
}


void Registers::print_registers() {
    char buff[128];
    for ( auto i : urange( 22 ) ) {
        uc_reg_read( static_cast<uc_engine *>( internal_uc ), Registers::regs_id[i], &reg );
        Log::reg.log("      %16s   %s", Registers::regs_names[i], to_hex(reg).c_str());
    }
}

void Registers::print_changed_registers() {
    for ( auto i : urange( 22 ) ) {
        uc_reg_read( static_cast<uc_engine *>( internal_uc ), Registers::regs_id[i], &reg );
        regs[i] = reg;
    }
    char buff[128];
    for ( auto i : urange( 22 ) ) {
        if ( Registers::regs_id[i] == UC_X86_REG_RIP && ( regs_old[i] != 0 ) )
            continue;
        if ( regs[i] != regs_old[i] ) {
            Log::new_val.log("      %16s   %s", Registers::regs_names[i], to_hex(regs[i]).c_str());
        }
    }
    for ( auto i : urange( 22 ) )
        regs_old[i] = regs[i];
}

ulong Registers::get_eax() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_EAX, &reg );
    return reg;
}

ulong Registers::get_ebx() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_EBX, &reg );
    return reg;
}

ulong Registers::get_ecx() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_ECX, &reg );
    return reg;
}

ulong Registers::get_edx() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_EDX, &reg );
    return reg;
}

ulong Registers::get_rax() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RAX, &reg );
    return reg;
}

ulong Registers::get_rbx() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RBX, &reg );
    return reg;
}

ulong Registers::get_rcx() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RCX, &reg );
    return reg;
}

ulong Registers::get_rdx() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RDX, &reg );
    return reg;
}

ulong Registers::get_r8() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_R8, &reg );
    return reg;
}

ulong Registers::get_r9() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_R9, &reg );
    return reg;
}

double Registers::get_xmm0() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM0, mini_buff );
    return *( double * )mini_buff;
}

double Registers::get_xmm1() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM1, mini_buff );
    return *( double * )mini_buff;
}

double Registers::get_xmm2() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM2, mini_buff );
    return *( double * )mini_buff;
}

double Registers::get_xmm3() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM3, mini_buff );
    return *( double * )mini_buff;
}

float Registers::get_xmm0_f() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM0, mini_buff );
    return *( float * )mini_buff;
}

float Registers::get_xmm1_f() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM1, mini_buff );
    return *( float * )mini_buff;
}

float Registers::get_xmm2_f() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM2, mini_buff );
    return *( float * )mini_buff;
}

float Registers::get_xmm3_f() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM3, mini_buff );
    return *( float * )mini_buff;
}

ulong Registers::get_rsp() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RSP, &reg );
    return reg;
}

ulong Registers::get_rsi() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RSI, &reg );
    return reg;
}

ulong Registers::get_rdi() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RDI, &reg );
    return reg;
}

ulong Registers::get_rip() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RIP, &reg );
    return reg;
}

ulong Registers::get_gs() {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_GS, &reg );
    return reg;
}

void Registers::get_gdtr( void *gdtr ) {
    uc_reg_read( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_GDTR, gdtr );
}

void Registers::set_eax( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_EAX, &reg );
}

void Registers::set_ebx( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_EBX, &reg );
}

void Registers::set_ecx( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_ECX, &reg );
}

void Registers::set_edx( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_EDX, &reg );
}

void Registers::set_rax( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RAX, &reg );
}

void Registers::set_rbx( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RBX, &reg );
}

void Registers::set_rcx( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RCX, &reg );
}

void Registers::set_rdx( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RDX, &reg );
}

void Registers::set_r8( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_R8, &reg );
}
void Registers::set_r9( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_R9, &reg );
}

void Registers::set_xmm0( double val ) {

    *( double * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM0, mini_buff );
}

void Registers::set_xmm1( double val ) {
    *( double * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM1, mini_buff );
}

void Registers::set_xmm2( double val ) {
    *( double * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM2, mini_buff );
}

void Registers::set_xmm3( double val ) {
    *( double * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM3, mini_buff );
}

void Registers::set_xmm0_f( float val ) {
    *( float * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM0, mini_buff );
}

void Registers::set_xmm1_f( float val ) {
    *( float * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM1, mini_buff );
}

void Registers::set_xmm2_f( float val ) {
    *( float * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM2, mini_buff );
}

void Registers::set_xmm3_f( float val ) {
    *( float * )mini_buff = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_XMM3, mini_buff );
}

void Registers::set_rsp( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RSP, &reg );
}

void Registers::set_rsi( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RSI, &reg );
}

void Registers::set_rdi( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RDI, &reg );
}

void Registers::set_rip( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_RIP, &reg );
}

void Registers::set_gs( ulong val ) {
    reg = val;
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_GS, &reg );
}

void Registers::set_gdtr( void *gdtr ) {
    uc_reg_write( static_cast<uc_engine *>( internal_uc ), UC_X86_REG_GDTR, gdtr );
}
