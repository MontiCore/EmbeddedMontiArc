/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "computer/system_calls.h"
#include "computer/computer_layout.h"
#include "computer.h"
#include "utility/debug.h"

void SystemCalls::init( Computer &computer ) {
    this->computer = &computer;
    sys_calls.resize( ComputerLayout::SYSCALLS_RANGE );
    section = &computer.memory.new_section( MemoryRange( ComputerLayout::SYSCALLS_ADDRESS, ComputerLayout::SYSCALLS_RANGE ),
                                            "SYSCALLS",
                                            "System",
                                            true, false, false );
    section->init_annotations();
    sys_call_pos = 0;
    syscall_stack.init( section );
}

void SystemCalls::handle_call( ulong addr ) {
    //Use memory annotation system to find external procedure.
    auto note_ptr = section->annotations.get_annotation( addr );
    if ( note_ptr == nullptr || note_ptr->type != Annotation::Type::FUNC ) {
        Log::err.log_tag("Instruction pointer at invalid system call address");
        //Execution will stop and return an error since the syscall section does not allow code execution.
        return;
    }
    auto &note = *note_ptr;
    auto &call = sys_calls[( uint )note.param];
    computer->debug.debug_syscall( call, note.param );
    if ( !call.supported() || !call.callback( *computer ) ) {
        if ( computer->debug.unsupported_syscalls() )
            computer->debug.debug_unsupp_syscall( call );
        computer->os->set_return_64( 0 ); //No external syscall registered or syscall error.
    }
    
    if ( !computer->was_stopped() ) { //Do not change stack and instruction pointers if exit() was called on the unicorn engine (it cancels uc_emu_stop())
        //Return to code (simulate 'ret' code)
        auto ret_address = computer->stack.pop_long();
        computer->registers.set_rip( ret_address ); //Set instruction pointer to popped address
    }
    
    //TODO add computer time
}

ulong SystemCalls::add_syscall( SysCall const &call, const char *reason ) {
    //Not using module names right now
    //std::string res_name = call.module + "!" + call.name;
    
    auto id = sys_call_pos++;
    auto proc_handle = syscall_stack.get_annotated_8byte( call.name, Annotation::Type::FUNC, id );
    
    sys_calls[id] = call;
    computer->debug.debug_register_syscall( call, section->address_range.get_local_index( proc_handle ), reason );
    
    computer->symbols.add_symbol( call.name, Symbols::Symbol::Type::SYSCALL, proc_handle );
    
    return proc_handle;
}
