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
    SysCall() : type( UNSUPPORTED ), addr( 0 ) {}
    SysCall( std::string const &name, std::string const &module,  SystemCallback const callback ) :
        type( callback != nullptr ? SUPPORTED : UNSUPPORTED ),
        name( name ), module( module ), callback( callback ), addr( 0 ) {}
    SysCall( std::string const &name, std::string const &module, ulong addr ) :
        type( LIBRARY_EXPORT ), name( name ), module( module ), callback( nullptr ), addr( addr ) {}
};

struct SystemCalls {
    MemorySection *section;
    SectionStack syscall_stack;
    
    
    ComputerDebug *debug;
    
    
    Array<SysCall> sys_calls;
    uint64_t sys_call_pos;
    
    SystemCalls() : section( nullptr ), sys_calls( ComputerLayout::SYSCALLS_RANGE ) {}
    
    bool loaded() {
        return section != nullptr;
    }
    
    void init( Memory &mem, ComputerDebug &debug );
    
    ulong get_syscall( const std::string &mod, const std::string &name );
    ulong add_syscall( SysCall const &call );
};