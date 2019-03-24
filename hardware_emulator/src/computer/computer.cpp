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
        Log::err << Log::tag << "Failed on uc_open(), returned error: " << internal->err << "\n";
        internal->uc = nullptr;
        return;
    }
    
    
    decoder.init( memory, time );
    
    memory.init( internal->uc );
    registers.init( internal->uc );
    
    debug.init( memory, registers, decoder );
    handles.init( memory );
    
    heap.init( memory, handles );
    stack.init( memory, registers );
    sys_calls.init( *this );
    
    mem_model.init( memory, time );
    
    uc_hook_add( internal->uc, &internal->trace1, UC_HOOK_CODE, ( void * )Computer::hook_code, this, 1, 0 );
    uc_hook_add( internal->uc, &internal->trace2, UC_HOOK_MEM_VALID, ( void * )Computer::hook_mem, this, 1, 0 );
    uc_hook_add( internal->uc, &internal->trace3, UC_HOOK_MEM_INVALID, ( void * )Computer::hook_mem_err, this, 1, 0 );
    
    exit_code_addr = sys_calls.add_syscall( SysCall( "exit", "SYSTEM", exit_callback ), "Computer" );
    
    
}

void Computer::drop() {
    if ( loaded() ) {
        os.reset();
        uc_close( internal->uc );
        delete internal;
        internal = nullptr;
    }
}


bool Computer::call( ulong address, const char *name ) {
    debug.debug_call( address, name );
    //Log::info << name << "()\n";
    stopped = false;
    stack.push_long( exit_code_addr );
    internal->err = uc_emu_start( internal->uc, address, 0xFFFFFFFFFFFFFFFF, 0, 0 );
    if ( internal->err ) {
        Log::err << Log::tag << "Failed on uc_emu_start() with error returned "
                 << internal->err << ": " << uc_strerror( internal->err ) << "\n";
        return false;
    }
    return true;
}

void Computer::set_os( OS::OS *os ) {
    this->os = std::unique_ptr<OS::OS>( os );
    os->init( *this );
}

void Computer::cb_code( ulong addr, uint size ) {
    if ( sys_calls.is_syscall( addr ) )
        sys_calls.handle_call( addr );
    else {
        auto ticks = decoder.handle_instruction( addr, size );
        auto m_time = mem_model.handle_access( MemAccess::FETCH, addr );
        debug.debug_code( addr, size, ticks, time.tick_time_pico * ticks + m_time );
    }
}

void Computer::cb_mem( MemAccess type, ulong addr, uint size, slong value ) {
    uint sec_id = memory.section_lookup[MemoryRange( addr, 1 )];
    ulong time = mem_model.handle_access( type, addr );
    debug.debug_mem( type, addr, size, value, time );
}

void Computer::cb_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong value ) {
    debug.debug_mem_err( type, err, addr, size, value );
}




void Computer::hook_code( void *uc, ulong addr, uint size, void *data ) {
    static_cast<Computer *>( data )->cb_code( addr, size );
}

bool Computer::hook_mem( void *uc, uint type, ulong addr, uint size, slong value, void *data ) {
    static_cast<Computer *>( data )->cb_mem( Memory::get_mem_access( type ), addr, size, value );
    return false;
}

bool Computer::hook_mem_err( void *uc, uint type, ulong addr, uint size, slong value, void *data ) {
    static_cast<Computer *>( data )->cb_mem_err( Memory::get_mem_access( type ), Memory::get_mem_err( type ), addr, size,
            value );
    return false;
}


bool Computer::exit_callback( Computer &inter ) {
    inter.exit_emulation();
    return true;
}

void Computer::exit_emulation() {
    uc_emu_stop( internal->uc );
    stopped = true;
}
