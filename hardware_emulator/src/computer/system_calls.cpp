#include "computer/system_calls.h"
#include "computer/computer_layout.h"
#include "debug.h"

void SystemCalls::init( Memory &mem, ComputerDebug &debug ) {
    this->debug = &debug;
    section = &mem.new_section();
    section->init( MemoryRange( ComputerLayout::SYSCALLS_ADDRESS, ComputerLayout::SYSCALLS_RANGE ), "SYSCALLS", "System",
                   true, false, false );
    section->init_annotations();
    sys_calls.init( ComputerLayout::SYSCALLS_RANGE );
    sys_call_pos = 0;
    syscall_stack.init( section );
}

ulong SystemCalls::get_syscall( const std::string &mod, const std::string &name ) {
    std::string res_name = mod + "!" + name;
    auto note = section->annotations.annotations->get_annotation( res_name );
    if ( note.type == Annotation::PROC )
        return note.base;
    else if ( note.type == Annotation::LIBRARY_EXPORT )
        return note.param;
    return 0;
}

ulong SystemCalls::add_syscall( SysCall const &call ) {
    std::string res_name = call.module + "!" + call.name;
    
    auto proc_handle = syscall_stack.get_8byte_slot();
    auto id = sys_call_pos++;
    
    sys_calls[id] = call;
    
    if ( call.type != SysCall::LIBRARY_EXPORT ) {
        section->annotations.add_annotation( proc_handle, Annotation( res_name, Annotation::PROC, id ) );
        debug->debug_register_syscall( call, section->address_range.get_local_index( proc_handle ) );
        return proc_handle;
    }
    else {
        section->annotations.add_annotation( proc_handle, Annotation( res_name, Annotation::LIBRARY_EXPORT, call.addr ) );
        return call.addr;
    }
    
}
