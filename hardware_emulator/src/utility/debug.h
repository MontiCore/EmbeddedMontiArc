/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <cstddef>
#include "computer/memory.h"
#include "computer/registers.h"
#include "computer/system_calls.h"
#include "computer/instruction_time.h"

#include <Zydis/Zydis.h>


/*
    The ComputerDebug component gets called by different part of a running emulator.
    The following flags can be changed to enable or disable a particular debug output type:
        debug                       Enables/Disables all outputs.
        d_code                      Show single instructions of the program.
        d_mem                       Show memory read and write events.
        d_regs                      Show the state of the registers after every instruction
        d_reg_update                Show only registers that were updated by an instruction.
        d_syscalls                  Show calls from the program to external functionalities.
        d_unsupported_syscalls      Show only the unsupported system calls.
        d_call                      Show function calls to the emulator (when emulation is started)

    The corresponding functions without the "d_" prefix say if a given output is enabled
    (combines the debug flag and the d_* flag)
*/
struct ComputerDebug {
    static constexpr uint BUFFER_SIZE = 0x400;
    Memory *memory;
    Registers *registers;
    CodeDecoder *decoder;
    ZydisFormatter formatter;
    
	std::vector<char> buffer;
    
    bool debug;
    bool d_code;
    bool d_mem;
    bool d_regs;
    bool d_syscalls;
    bool d_unsupported_syscalls;
    bool d_reg_update;
    bool d_call;
    
    ComputerDebug() : memory( nullptr ), registers( nullptr ), buffer( BUFFER_SIZE ), debug( false ), d_code( true ),
        d_mem( true ), d_unsupported_syscalls( true ),
        d_regs( false ), d_syscalls( true ), d_call( true ), d_reg_update( true ), decoder( nullptr ), formatter() {}
        
    void init( Memory &mem, Registers &regs, CodeDecoder &decoder );
    
    void debug_syscall( SysCall &sys_call, ulong id );
    void debug_unsupp_syscall( SysCall &sys_call );
    void debug_code( ulong addr, uint size, ulong ticks, ulong time );
    void debug_code_noval( ulong addr, uint size );
    void debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val );
    void debug_mem( MemAccess type, ulong addr, uint size, slong val, ulong time );
    void debug_call( ulong address, const char *name );
    
    void debug_register_syscall( SysCall const &call, ulong addr, const char *reason );
    
    bool code() {
        return debug && d_code;
    }
    bool mem() {
        return debug && d_mem;
    }
    bool regs() {
        return debug && d_regs;
    }
    bool syscalls() {
        return debug && d_syscalls;
    }
    bool unsupported_syscalls() {
        return debug && d_unsupported_syscalls;
    }
    bool reg_update() {
        return debug && d_reg_update;
    }
    bool call() {
        return debug && d_call;
    }
};
