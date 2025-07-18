/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "memory.h"
#include "caching.h"
#include "registers.h"
#include "utility/debug.h"
#include "system_calls.h"
#include "function_calling.h"
#include "instruction_time.h"
#include "symbols.h"
#include "os.h"
#include "json.hpp"
#include "os_windows/windows_fast_call.h"
#include "os_linux/linux_fast_call.h"
using json = nlohmann::json;


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
        OS::WindowsFastCall func_call_windows;
        OS::LinuxFastCall func_call_linux;

        ComputerDebug debug;
        
        ComputerTime time;
        // represents the memory hierarchy
        MemoryModel mem_model;
        
        MemoryRange io_slot;


        bool uses_shadow_space = false;
        ulong walk_table_pos = 0;
        ulong walk_table_end = 0;

        ulong throw_error_addr = 0;
        ulong print_cout_addr = 0;
        ulong print_cerr_addr = 0;
        
        Computer() : internal( nullptr ), func_call_windows(registers, memory), func_call_linux(registers) {}
        ~Computer() {
            drop();
        }
        
        void init();
        void drop();
        
        bool loaded() {
            return internal != nullptr;
        }
        
        void call( ulong address, const char *name );
        // When making a callback to a virtual function inside a syscall hook
        void call_inside(ulong address, ulong return_addr);
        // Call in the AFTER syscall callback to cleanup the stack if needed
        void call_inside_after();
        
        void set_os( OS::OS *os );
        
        ulong add_symbol_handle(const char *name);
        
        
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
        
        InternalComputer *internal = nullptr;
        ulong exit_code_addr = 0;
        bool stopped = false;
        bool did_throw = false;
        std::string autopilot_throw_msg;

        const char* unicorn_error();
};



/*
    CacheSettings is used to configure the cache layout of the computer.
    Replace the IL1, DL1, L2 and L3 members with Cache objects to enable and configure the given cache level.

    if the next command of the MessageParser used to read the configuration of the autopilot starts with "cache_",
    handle_config() will read the config entry and set the according cache settings.

    setup_computer() will then apply the configuration to the computer.
*/
struct CacheSettings {
    enum class CacheType {
        SHARED,
        I,
        D
    };
    struct Cache {
        CacheType type;
        int level;
        int way_count;
        bool used;
        //Pico second times
        uint write_ticks;
        uint read_ticks;
        uint line_length;
        ulong size; //Size is number of block entries
        Cache() : used( false ), write_ticks(0), read_ticks(0), line_length(64), size(0), level(0), type(CacheType::SHARED) {}
        /*Cache( CacheType type, int level, uint read_ticks, uint write_ticks, uint size, uint line_length = 8 ) :
            type(type), level(level),
            write_ticks( write_ticks ), read_ticks( read_ticks ), size( size ), line_length(line_length), used( true ) {}*/
    };
    std::vector<std::unique_ptr<Cache>> caches;
    
    void handle_config(const json& settings);
    
    void setup_computer( Computer &computer );
    
};