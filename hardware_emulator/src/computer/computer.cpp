#include "computer/computer.h"
#include "unicorn/unicorn.h"

struct InternalComputer {
    uc_engine *uc;
    uc_err err;
    uc_hook trace1, trace2, trace3, trace4;
};

void Computer::init() {
    drop();
    internal = new InternalComputer();
    
    internal->err = uc_open( UC_ARCH_X86, UC_MODE_64, &internal->uc );
    if ( internal->err != UC_ERR_OK ) {
        printf( "Failed on uc_open() with error returned: %u\n", internal->err );
        internal->uc = nullptr;
        return;
    }
    
    ZydisDecoderInit( &decoder.decoder, ZYDIS_MACHINE_MODE_LONG_64, ZYDIS_ADDRESS_WIDTH_64 );
    
    memory.init( internal->uc );
    registers.init( internal->uc );
    
    debug.init( memory, registers, decoder );
    handles.init( memory );
    
    heap.init( memory, handles );
    stack.init( memory, registers );
    sys_calls.init( memory, debug, symbols );
    
    uc_hook_add( internal->uc, &internal->trace1, UC_HOOK_CODE, ( void * )Computer::hook_code, this, 1, 0 );
    uc_hook_add( internal->uc, &internal->trace2, UC_HOOK_MEM_VALID, ( void * )Computer::hook_mem, this, 1, 0 );
    uc_hook_add( internal->uc, &internal->trace3, UC_HOOK_MEM_INVALID, ( void * )Computer::hook_mem_err, this, 1, 0 );
    
    exit_code_addr = sys_calls.add_syscall( SysCall( "exit", "SYSTEM", exit_callback ) );
    
}

void Computer::drop() {
    if ( loaded() ) {
        uc_close( internal->uc );
        delete internal;
        internal = nullptr;
    }
}


bool Computer::call( ulong address, const char *name ) {
    //Log::info << name << "()\n";
    stopped = false;
    stack.push_long( exit_code_addr );
    internal->err = uc_emu_start( internal->uc, address, 0xFFFFFFFFFFFFFFFF, 0, 0 );
    if ( internal->err ) {
        Log::err << Log::tag << "Failed on uc_emu_start() with error returned "
                 << internal->err
                 << ": "
                 << uc_strerror( internal->err )
                 << "\n";
        return false;
    }
    return true;
}

void Computer::set_os( OS::OS *os ) {
    this->os = std::unique_ptr<OS::OS>( os );
    os->init( *this );
}

void Computer::cb_code( ulong addr, uint size ) {

    uint computer_time = 1000;
    /*
        Check if the instruction is in the SystemCalls memory range. If so, it means call (ASM) was called
        with a registered sytem call function address.
    */
    if ( sys_calls.section->address_range.contains( addr ) ) {
        //Use memory annotation system to find external procedure.
        auto note_ptr = sys_calls.section->annotations.get_annotation( addr );
        if ( note_ptr == nullptr || note_ptr->type != Annotation::FUNC ) {
            std::cerr << "Instruction pointer in invalid PROC region" << std::endl;
            return;
        }
        auto &note = *note_ptr;
        auto &call = sys_calls.sys_calls[( uint )note.param];
        debug.debug_syscall( call, note.param );
        if ( call.type != SysCall::SUPPORTED || !call.callback( *this, call ) )
            func_call->set_return( 0 ); //No external syscall registered or syscall error.
            
        if ( !stopped ) { //Do not change stack and instruction pointers if exit() was called on the unicorn engine (it cancels uc_emu_stop())
            //Return to code (simulate 'ret' code)
            auto ret_address = stack.pop_long();
            registers.set_rip( ret_address ); //Set instruction pointer to popped address
        }
        
        //TODO add computer time
    }
    else {
        decoder.length = size;
        decoder.code = ( uchar * )memory.read_memory( addr, size );
        decoder.succeeded = ZYAN_SUCCESS(
                                ZydisDecoderDecodeBuffer( &decoder.decoder, decoder.code, decoder.length, &decoder.instruction )
                            );
        if ( decoder.succeeded )
            computer_time = get_instruction_ticks( decoder.instruction );
            
        //if ( computer_time == 1000 )
        debug.debug_code( addr, size );
    }
    //computing_time += computer_time;
}

void Computer::cb_mem( MemAccess type, ulong addr, uint size, slong value ) {
    debug.debug_mem( type, addr, size, value );
}

void Computer::cb_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong value ) {
    debug.debug_mem_err( type, err, addr, size, value );
}




void Computer::hook_code( void *uc, ulong addr, uint size, void *data ) {
    static_cast<Computer *>( data )->cb_code( addr, size );
}

bool Computer::hook_mem( void *uc, uint type, ulong addr, uint size, slong value, void *data ) {
    static_cast<Computer *>( data )->cb_mem( get_mem_access( type ), addr, size, value );
    return false;
}

bool Computer::hook_mem_err( void *uc, uint type, ulong addr, uint size, slong value, void *data ) {
    static_cast<Computer *>( data )->cb_mem_err( get_mem_access( type ), get_mem_err( type ), addr, size, value );
    return false;
}


MemAccess Computer::get_mem_access( uint type ) {
    switch ( type ) {
        case UC_MEM_WRITE: case UC_MEM_WRITE_UNMAPPED: case UC_MEM_WRITE_PROT:
            return MemAccess::WRITE;
        case UC_MEM_READ: case UC_MEM_READ_UNMAPPED: case UC_MEM_READ_PROT:
            return MemAccess::READ;
        case UC_MEM_FETCH: case UC_MEM_FETCH_UNMAPPED: case UC_MEM_FETCH_PROT:
            return MemAccess::FETCH;
        default:
            return MemAccess::NONE;
    }
}

MemAccessError Computer::get_mem_err( uint type ) {
    switch ( type ) {
        case UC_MEM_WRITE: case UC_MEM_READ: case UC_MEM_FETCH:
            return MemAccessError::NONE;
        case UC_MEM_WRITE_UNMAPPED: case UC_MEM_READ_UNMAPPED: case UC_MEM_FETCH_UNMAPPED:
            return MemAccessError::MAPPED;
        case UC_MEM_WRITE_PROT: case UC_MEM_READ_PROT: case UC_MEM_FETCH_PROT:
            return MemAccessError::PROT;
        default:
            return MemAccessError::NONE;
    }
}

bool Computer::exit_callback( Computer &inter, SysCall &syscall ) {
    inter.exit_emulation();
    return true;
}

void Computer::exit_emulation() {
    uc_emu_stop( internal->uc );
    stopped = true;
}
