/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "memory.h"
#include "computer_layout.h"
#include "symbols.h"

struct Computer;
struct SysCall;
struct ComputerDebug;

// function pointer
using SystemCallback = bool( * )( Computer &computer );


/*
    A SysCall represents a System/OS function that is not in the actual memory of the emulator.
    It has a module (ex. KERNEL32.DLL) and a symbol name (can be a decorated name).
    It can have an optional callback method registered, which is an external implementation of
    the named function.
*/
struct SysCall {
    std::string name;
    std::string module;
    SystemCallback callback;
    SysCall() : callback( nullptr ) {}
    SysCall( std::string const &name, std::string const &module,  SystemCallback const callback ) :
        name( name ), module( module ), callback( callback ) {}
    bool supported() {
        return callback != nullptr;
    }
};

/*
    Collection of registered SysCalls.
    Use get_syscall() to get a registered SysCall by module and name.
    Use add_syscall() to register a SysCall.
    Both return the virtual address of the function handle.

    SysCalls can be registered without a callback managing the function. These are called
    "unsupported system calls" and do nothing except returning 0.
*/
struct Computer;
struct SystemCalls {
    Computer *computer;
    
    
    MemorySection *section;
    SectionStack syscall_stack;
    
	std::vector<SysCall> sys_calls;
    uint sys_call_pos;

    ulong after_pthread_once_function = 0;
    ulong after_initterm = 0;
    
    SystemCalls() : section( nullptr ), computer( nullptr ), sys_call_pos(0) {}
    
    bool loaded() {
        return section != nullptr;
    }
    
    void init( Computer &computer );
    
    inline bool is_syscall( ulong addr ) {
        /*
        Check if the instruction is in the SystemCalls memory range. If so, it means 'call' (ASM) was called
        with a registered sytem function address.
        */
        return section->address_range.contains( addr );
    }
    
    void handle_call( ulong addr );
    
    ulong add_syscall( SysCall const &call, const char *reason );
};
