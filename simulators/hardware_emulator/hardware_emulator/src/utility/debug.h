/**
 * (c) https://github.com/MontiCore/monticore
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
        d_instruction_operands      Show operands of executed instructions.
        d_operands_details          Show details of the operands. Works only if 'd_instruction_operands' is set tu 'true'
        d_cache_hit_ratio           Show cache accesses/hits/misses counts at the end
        d_mem_access              Show memory hierarchy access information of memory access operations

    The corresponding functions without the "d_" prefix say if a given output is enabled
    (combines the debug flag and the d_* flag)
*/
struct ComputerDebug {
    //static constexpr uint BUFFER_SIZE = 0x400;
    Memory *memory;
    Registers *registers;
    CodeDecoder *decoder;
    //ZydisFormatter formatter;
    
	//std::vector<char> buffer;
    
    bool debug = false;
    bool d_code = false;
    bool d_mem = false;
    bool d_regs = false;
    bool d_syscalls = false;
    bool d_unsupported_syscalls = false;
    bool d_reg_update = false;
    bool d_call = false;
    bool d_instruction_operands = false;
    bool d_operands_details = false;
    bool d_cache_hit_ratio = false;
    bool d_mem_access = false;
    
    ComputerDebug() : memory( nullptr ), registers( nullptr ), decoder( nullptr ) {}
        
    void init( Memory &mem, Registers &regs, CodeDecoder &decoder );
    
    void debug_syscall( SysCall &sys_call, ulong id );
    void debug_unsupp_syscall( SysCall &sys_call );
    void debug_code( ulong addr, uint size, ulong ticks, ulong time );
    void debug_code_noval( ulong addr, uint size );
    void debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val );
    void debug_mem( MemAccess type, ulong addr, uint size, slong val, ulong time );
    void debug_call( ulong address, const char *name );
    void debug_instruction_operands();
    void debug_operands_details(ZydisDecodedOperand &operand);
    void debug_cache_hit_ratio( MemoryModel *mem_model );
    void debug_memory_access(MemoryModel *mem_model);
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

    bool instruction_operands() {
        return debug && d_instruction_operands;
    }

    bool cache_hit_ratio() {
        return debug && d_cache_hit_ratio;
    }
     
    bool memory_access() {
        return debug && d_mem_access;
    }


    static std::basic_string<char> resolve_operand_type(CodeDecoder* decoder, ComputerDebug* debug, ZydisOperandType type);
    static std::basic_string<char> resolve_operand_element_type(CodeDecoder* decoder, ComputerDebug* debug, ZydisElementType element_type);
    static std::basic_string<char> resolve_operand_size(CodeDecoder* decoder, ZyanU16 size);
    static std::basic_string<char> resolve_element_size(CodeDecoder* decoder, ZydisElementSize size);
};
