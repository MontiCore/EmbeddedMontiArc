/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "memory.h"
#include "registers.h"
#include "debug.h"
#include "system_calls.h"
#include "function_calling.h"
#include "instruction_time.h"
#include "symbols.h"
#include "os.h"
#include "caching.h"


struct InternalComputer;



/**
    The Computer structure is a representation of an OS-less virtual computer.

    For the overall structure see "docs/computer_os.png".

    It handles the creation of the Unicorn engine, which emulates virtual memory and cpu instructions.
    The Computer has hooks registered in the emulator for Code (every instruction before their execution),
    for memory access and invalid access (R/W).

    It initiates the following subcomponents:
        VirtualStack
        VirtualHeap
        Handles
        SystemCalls
        Memory
        Registers
        CodeDecoder
        ComputerDebug
        ComputerTime
        MemoryModel


    An operating system model implementing the OS::OS interface can be hooked to the computer using set_os()
    This will take ownership of the given os object (handles its destruction) and will call its initialization function

    The call() method is the entry point used to start the emulation at a given address.

*/
struct Computer {
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
        MemoryModel mem_model;
        
        
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
        
        
        
        //Calling this from within a callback will stop the simulation after the callback returns.
        void exit_emulation();
        bool was_stopped() {
            return stopped;
        }
        
    private:
    
        static void hook_code( void *uc, ulong addr, uint size, void *data );
        static bool hook_mem( void *uc, uint type, ulong addr, uint size, slong value, void *data );
        static bool hook_mem_err( void *uc, uint type, ulong addr, uint size, slong value, void *data );
        
        //Callbacks that get called for instructions and memory access by the Unicorn engine.
        void cb_code( ulong addr, uint size );
        void cb_mem( MemAccess type, ulong addr, uint size, slong value );
        void cb_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong value );
        
        static bool exit_callback( Computer &inter );
        
        InternalComputer *internal;
        ulong exit_code_addr;
        bool stopped;
};
