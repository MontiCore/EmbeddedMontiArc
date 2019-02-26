#pragma once
#include <cstddef>
#include "computer/memory.h"
#include "computer/registers.h"
#include "computer/system_calls.h"
#include "computer/instruction_time.h"

#include <Zydis/Zydis.h>

struct ComputerDebug {
    static constexpr uint BUFFER_SIZE = 0x400;
    Memory *mem;
    Registers *regs;
    CodeDecoder *decoder;
    ZydisFormatter formatter;
    
    Array<char> buffer;
    
    bool debug;
    bool d_code;
    bool d_mem;
    bool d_regs;
    bool d_syscalls;
    bool d_reg_update;
    
    ComputerDebug() : mem( nullptr ), regs( nullptr ), buffer( BUFFER_SIZE ), debug( true ), d_code( true ), d_mem( true ),
        d_regs( false ), d_syscalls( true ), d_reg_update( true ), decoder( nullptr ) {}
        
    void init( Memory &mem, Registers &regs, CodeDecoder &decoder );
    
    void debug_syscall( SysCall &sys_call, ulong id );
    void debug_code( ulong addr, uint size );
    void debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val );
    void debug_mem( MemAccess type, ulong addr, uint size, slong val );
    
    void debug_register_syscall( SysCall const &call, ulong addr );
};