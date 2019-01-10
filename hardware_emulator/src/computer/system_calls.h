#pragma once
#include "memory.h"
#include "computer_layout.h"

struct Computer;
struct SysCall;
struct ComputerDebug;

using SystemCallback = bool( * )( Computer &, SysCall &syscall );

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

struct SystemCalls {
    MemorySection *section;
    SectionStack syscall_stack;
    
    
    ComputerDebug *debug;
    
    
    Array<SysCall> sys_calls;
    uint sys_call_pos;
    
    SystemCalls() : section( nullptr ) {}
    
    bool loaded() {
        return section != nullptr;
    }
    
    void init( Memory &mem, ComputerDebug &debug );
    
    ulong get_syscall( const std::string &mod, const std::string &name );
    ulong add_syscall( SysCall const &call );
};
