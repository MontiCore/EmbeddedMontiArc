#pragma once
#include "memory.h"
#include "registers.h"
#include "debug.h"
#include "system_calls.h"
#include "function_calling.h"
#include "instruction_time.h"
#include "symbols.h"
#include "os.h"

#include <Zydis/Zydis.h>

struct InternalComputer;

struct ComputerTime {
    bool use_time;
    ulong pico_time;
    ulong micro_time;
    ulong cpu_frequency;
    ulong tick_time_pico;
    
    ComputerTime() : micro_time( 0 ), cpu_frequency( 1000000 ), pico_time( 0 ), tick_time_pico( 0 ), use_time( false ) {}
    
    void set_frequency( ulong cpu_frequency ) {
        this->cpu_frequency = cpu_frequency;
        this->tick_time_pico = 1000000000000UL / cpu_frequency;
    }
    
    void add_ticks( ulong tick_count ) {
        if ( use_time )
            add_pico_time( tick_time_pico * tick_count );
    }
    
    void add_pico_time( ulong delta ) {
        if ( use_time ) {
            pico_time += delta;
            if ( pico_time >= 1000000 ) {
                micro_time += pico_time / 1000000;
                pico_time %= 1000000;
            }
        }
    }
    
    void reset() {
        pico_time = 0;
        micro_time = 0;
    }
};

/**
    @class Computer
    The Computer structure is a representation of our OS-less virtual computer.
    It handles the creation of the Unicorn engine, which emulates virtual memory and cpu instructions.
    Computer has hooks into the emulator for Code (every instruction before their execution),
    for memory access and invalid access (R/W) (only used for debugging).
    It initiates the VirtualStack, VirtualHeap, Memory, Handles and SystemCalls stuctures.

    The Memory structure provides an interface to intiate virtual memory sections and to annotate
    them (for debugging and handle managment).

    The Handle structure creates a section in virtual memory where emulated system functions can allocate
    dummy handles to give back to the calling code (the *address* of these dummies is returned, allowing to
    catch a potential memory access is the handle section).

    SystemCalls works in a similar way as Handles: a reserved virtual memory section where virtual addresses of
    functions are allocated. The Unicorn code hook implemented by Computer checks for any code execution in the
    system call section, intercepting it, checking for a manual implementation of the given call and simulating
    the return to the calling function.

    VirtualStack is the isolated virtual memory section reserved for the program stack, with wrapper functions
    allowing to simulate pop/push from outside the emulator (for manual reading/placing of data).

    VirtualHeap is a simple implementation of a heap used by the alloc/free system call functions.

    The ComputerDebug sturcture contains functions to display register changes, memory W/R and dissasembled code.
    The different type of debug outputs can be switched on/off.

    The Code hook is used to count or approximate the number of CUP ticks used by the instructions using the
    get_instruction_ticks() method.


    Computer gives the entry point for code emulation through the call() method.

    @see call()
    @see Memory
    @see Registers
    @see FastCall

*/
struct Computer {

    InternalComputer *internal;
    
    Symbols symbols;
    
    VirtualHeap heap;
    VirtualStack stack;
    Handles handles;
    SystemCalls sys_calls;
    
    Memory memory;
    Registers registers;
    
    CodeDecoder decoder;
    
    std::unique_ptr<OS::OS> os;
    std::unique_ptr<FunctionCalling> func_call;
    
    ComputerDebug debug;
    
    ComputerTime time;
    
    MemoryRange io_slot;
    
    Computer() : internal( nullptr ) {}
    ~Computer() {
        drop();
    }
    
    void init();
    void drop();
    
    bool loaded() {
        return internal != nullptr;
    }
    
    bool call( ulong address, const char *name );
    
    void set_os( OS::OS *os );
    
    static void hook_code( void *uc, ulong addr, uint size, void *data );
    static bool hook_mem( void *uc, uint type, ulong addr, uint size, slong value, void *data );
    static bool hook_mem_err( void *uc, uint type, ulong addr, uint size, slong value, void *data );
    
    void cb_code( ulong addr, uint size );
    void cb_mem( MemAccess type, ulong addr, uint size, slong value );
    void cb_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong value );
    
    static MemAccess get_mem_access( uint type );
    static MemAccessError get_mem_err( uint type );
    
    static bool exit_callback( Computer &inter, SysCall &syscall );
    void exit_emulation();
    ulong exit_code_addr;
    bool stopped;
};