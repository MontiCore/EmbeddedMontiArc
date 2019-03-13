#include "computer/system_calls.h"
#include "computer/computer_layout.h"
#include "debug.h"

void SystemCalls::init( Memory &mem, ComputerDebug &debug, Symbols &symbols ) {
    sys_calls.init( ComputerLayout::SYSCALLS_RANGE );
    this->debug = &debug;
    this->symbols = &symbols;
    section = &mem.new_section();
    section->init( MemoryRange( ComputerLayout::SYSCALLS_ADDRESS, ComputerLayout::SYSCALLS_RANGE ), "SYSCALLS", "System",
                   true, false, false );
    section->init_annotations();
    sys_call_pos = 0;
    syscall_stack.init( section );
}

ulong SystemCalls::add_syscall( SysCall const &call, const char *reason ) {
    //Not using module names right now
    //std::string res_name = call.module + "!" + call.name;
    
    auto id = sys_call_pos++;
    auto proc_handle = syscall_stack.get_annotated_8byte( call.name, Annotation::FUNC, id );
    
    sys_calls[id] = call;
    debug->debug_register_syscall( call, section->address_range.get_local_index( proc_handle ), reason );
    
    symbols->add_symbol( call.name, Symbols::Symbol::SYSCALL, proc_handle, id );
    
    return proc_handle;
}
