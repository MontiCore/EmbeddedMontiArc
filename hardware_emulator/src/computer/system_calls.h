#pragma once
#include "memory.h"
#include "computer_layout.h"
#include "symbols.h"

struct Computer;
struct SysCall;
struct ComputerDebug;

using SystemCallback = bool( * )( Computer &, SysCall &syscall );


/*
    A SysCall represents a System/OS function that is not in the actual memory of the emulator.
    It has a module (ex. KERNEL32.DLL) and a symbol name (can be a decorated name).
    It can have an optional callback method registered, which is an external implementation of
    the named function.
*/
struct SysCall {
    enum Type {
        UNSUPPORTED,
        SUPPORTED,
        LIBRARY_EXPORT
    };
    Type type;
    std::string name;
    std::string module;
    SystemCallback callback;
    ulong addr;
    void *user_data;
    SysCall() : type( UNSUPPORTED ), addr( 0 ), user_data( nullptr ) {}
    SysCall( std::string const &name, std::string const &module,  SystemCallback const callback,
             void *user_data = nullptr ) :
        type( callback != nullptr ? SUPPORTED : UNSUPPORTED ), user_data( user_data ),
        name( name ), module( module ), callback( callback ), addr( 0 ) {}
    SysCall( std::string const &name, std::string const &module, ulong addr, void *user_data = nullptr ) :
        type( LIBRARY_EXPORT ), name( name ), module( module ), callback( nullptr ), addr( addr ), user_data( user_data ) {}
};

/*
    Collection of registered SysCalls.
    Use get_syscall() to get a registered SysCall by module and name.
    Use add_syscall() to register a SysCall.
    Both return the virtual address of the function handle.
*/
struct SystemCalls {
    MemorySection *section;
    Symbols *symbols;
    SectionStack syscall_stack;
    
    
    ComputerDebug *debug;
    
    
    Array<SysCall> sys_calls;
    uint sys_call_pos;
    
    SystemCalls() : section( nullptr ), symbols( nullptr ) {}
    
    bool loaded() {
        return section != nullptr;
    }
    
    void init( Memory &mem, ComputerDebug &debug, Symbols &symbols );
    
    ulong add_syscall( SysCall const &call, const char *reason );
};
