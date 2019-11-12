/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
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
        internal->uc = nullptr;
        throw_error(Error::hardware_emu_init_error(std::string("uc_open(): ") + unicorn_error()));
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


void Computer::call( ulong address, const char *name ) {
    debug.debug_call( address, name );
    stopped = false;
    stack.push_long( exit_code_addr );
    internal->err = uc_emu_start( internal->uc, address, 0xFFFFFFFFFFFFFFFF, 0, 0 );
    if (internal->err)
        throw_error(std::string("Software emulation error: (call to uc_emu_start() ): \n\t")+ unicorn_error());
}

void Computer::set_os( OS::OS *os ) {
    this->os = std::unique_ptr<OS::OS>( os );
    os->init( *this );
}

void Computer::cb_code( ulong addr, uint size ) {
    if ( sys_calls.is_syscall( addr ) )
        sys_calls.handle_call( addr );
    else {
        ulong ticks;
        bool no_val = decoder.handle_instruction( addr, size, ticks );
        auto m_time = mem_model.handle_access( MemAccess::FETCH, addr );
        debug.debug_code( addr, size, ticks, time.cpu_tick_time_pico * ticks + m_time );
        if ( no_val )
            debug.debug_code_noval( addr, size );
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

const char* Computer::unicorn_error()
{
    return uc_strerror(internal->err);
}

void Computer::exit_emulation() {
    uc_emu_stop( internal->uc );
    stopped = true;
}




MemoryAccessInterface *setup_cache( Computer &computer, CacheSettings::Cache &cache ) {
    auto cache_layer = new FifoCache( computer.memory, cache.size );
    cache_layer->block_size = cache.block_size;
    cache_layer->set_ticks( computer.time, cache.read_ticks, cache.write_ticks );
    return cache_layer;
}

void CacheSettings::handle_config( MessageParser &parser ) {
    Cache *cache = nullptr;
    if ( parser.is_cmd( "cache_IL1" ) )
        cache = &IL1;
    else if ( parser.is_cmd( "cache_DL1" ) )
        cache = &DL1;
    else if ( parser.is_cmd( "cache_L2" ) )
        cache = &L2;
    else if ( parser.is_cmd( "cache_L3" ) )
        cache = &L3;
    else {
        Log::err << Log::tag << "Unknown cache level\n";
        return;
    }
    slong size;
    slong r_ticks;
    slong w_ticks;
    if ( parser.get_long( size ) ) {
        if ( size == 0 )
            *cache = Cache();
        else {
            if ( parser.get_long( r_ticks ) && parser.get_long( w_ticks ) )
                *cache = Cache( ( uint ) r_ticks, ( uint )w_ticks, ( uint )size );
            else
                Log::err << Log::tag << "Could not read tick parameters of cache config\n";
        }
    }
    else
        Log::err << Log::tag << "Could not read size parameter of cache config\n";
        
}

void CacheSettings::setup_computer( Computer &computer ) {
    if ( L3.used )
        computer.mem_model.add_common_interface( setup_cache( computer, L3 ) );
    if ( L2.used )
        computer.mem_model.add_common_interface( setup_cache( computer, L2 ) );
    if ( DL1.used )
        computer.mem_model.add_data_interface( setup_cache( computer, DL1 ) );
    if ( IL1.used )
        computer.mem_model.add_instruction_interface( setup_cache( computer, IL1 ) );
}
