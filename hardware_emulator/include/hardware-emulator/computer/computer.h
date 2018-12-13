#pragma once
#include "memory.h"
#include "registers.h"
#include "debug.h"
#include "system_calls.h"
#include "method_calling.h"

struct InternalComputer;

struct Computer {

    InternalComputer *internal;
    
    VirtualHeap heap;
    VirtualStack stack;
    Handles handles;
    SystemCalls sys_calls;
    
    Memory memory;
    Registers registers;
    
    ComputerDebug debug;
    
    FastCall fast_call;
    
    Computer() : internal( nullptr ), fast_call( registers ) {}
    ~Computer() {
        drop();
    }
    
    void init();
    void drop();
    
    bool loaded() {
        return internal != nullptr;
    }
    
    bool call( ulong address );
    
    static void hook_code( void *uc, ulong addr, uint size, void *data );
    static bool hook_mem( void *uc, uint type, ulong addr, uint size, slong value, void *data );
    static bool hook_mem_err( void *uc, uint type, ulong addr, uint size, slong value, void *data );
    
    void cb_code( ulong addr, uint size );
    void cb_mem( MemAccess type, ulong addr, uint size, slong value );
    void cb_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong value );
    
    static MemAccess get_mem_access( uint type );
    static MemAccessError get_mem_err( uint type );
};