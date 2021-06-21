/**
 * (c) https://github.com/MontiCore/monticore
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
    
    exit_code_addr = sys_calls.add_syscall(SysCall("exit", "EMU", [](Computer& computer) {
        computer.exit_emulation();
        return true;
    }), "Computer" );
    throw_error_addr = sys_calls.add_syscall(SysCall("EMU_throw_error", "EMU", [](Computer& computer) {
        auto type_str_addr = computer.os->get_param1_64();
        auto msg_str_addr = computer.os->get_param2_64();
        std::string type_str = computer.memory.read_str(type_str_addr);
        auto msg_str = computer.memory.read_str(msg_str_addr);
        computer.autopilot_throw_msg = "[" + type_str + "] " + msg_str;
        computer.did_throw = true;
        computer.exit_emulation();
        return true;
    }), "Computer");
    print_cout_addr = sys_calls.add_syscall(SysCall("EMU_print_cout", "EMU", [](Computer& computer) {
        auto msg_str_addr = computer.os->get_param1_64();
        auto msg_str = computer.memory.read_str(msg_str_addr);
        Log::ap.log_tag("[cout] %s", msg_str);
        return true;
    }), "Computer");
    print_cerr_addr = sys_calls.add_syscall(SysCall("EMU_print_cerr", "EMU", [](Computer& computer) {
        auto msg_str_addr = computer.os->get_param1_64();
        auto msg_str = computer.memory.read_str(msg_str_addr);
        Log::ap.log_tag("[cerr] %s", msg_str);
        return true;
    }), "Computer");
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
    did_throw = false;

    if (uses_shadow_space) {
        auto rsp = registers.get_rsp();
        registers.set_rsp(rsp - 32);
    }

    stack.push_long( exit_code_addr );
    internal->err = uc_emu_start( internal->uc, address, 0xFFFFFFFFFFFFFFFF, 0, 0 );
    if (internal->err)
        throw_error(std::string("Software emulation error: (call to uc_emu_start() ): \n\t")+ unicorn_error());
    if (uses_shadow_space) {
        auto rsp = registers.get_rsp();
        registers.set_rsp(rsp + 32);
    }
    if (did_throw) {
        throw_error("Throw called in autopilot: " + autopilot_throw_msg);
    }
}

void Computer::call_inside(ulong address, ulong return_addr)
{
    if (uses_shadow_space) {
        auto rsp = registers.get_rsp();
        registers.set_rsp(rsp - 32);
    }
    stack.push_long(return_addr);
    stack.push_long(address); // Just push the target address since 'sys_calls.handle_call()' always pops the next RIP address from the stack
}

void Computer::call_inside_after()
{
    if (uses_shadow_space) {
        auto rsp = registers.get_rsp();
        registers.set_rsp(rsp + 32);
    }
}

void Computer::set_os( OS::OS *os ) {
    this->os = std::unique_ptr<OS::OS>( os );
    os->init( *this );
    uses_shadow_space = os->uses_shadow_space();
}

ulong Computer::add_symbol_handle(const char* name)
{
    auto handle = handles.add_handle(name);
    symbols.add_symbol(name, Symbols::Symbol::Type::HANDLE, handle);
    return handle;
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


const char* Computer::unicorn_error()
{
    return uc_strerror(internal->err);
}

void Computer::exit_emulation() {
    uc_emu_stop( internal->uc );
    stopped = true;
}




MemoryAccessInterface *setup_cache( Computer &computer, CacheSettings::Cache &cache ) {
    return new NWayAssociativeCache(2, cache.size, cache.line_length, cache.read_ticks, cache.write_ticks, computer.time);
}

void CacheSettings::handle_config(const json& settings) {
    if (!settings.contains("caches")) return;
    auto& cache_settings = settings["caches"];
    if (!cache_settings.is_array()) return;
    for (auto &c : cache_settings){
        auto cache = new Cache();
        auto &type_config = c["type"];
        std::string type;
        if (json_get(c, "type", type)) {
            if (type.compare("shared") == 0) cache->type = CacheType::SHARED;
            else if (type.compare("I") == 0) cache->type = CacheType::I;
            else if (type.compare("D") == 0) cache->type = CacheType::D;
            else Log::err.log_tag("Unknown Cache Type: %s", type.c_str());
        }
        json_get(c, "level", cache->level);
        json_get(c, "size", cache->size);
        json_get(c, "read_ticks", cache->read_ticks);
        json_get(c, "write_ticks", cache->write_ticks);
        json_get(c, "line_length", cache->line_length);
        cache->used = true;
        caches.emplace_back(cache);
    } 
}

void CacheSettings::setup_computer( Computer &computer ) {
    std::vector<Cache*> cache_list(caches.size());
    for (auto i : range(caches.size())){
        cache_list[i] = caches[i].get();
    }
    std::sort(cache_list.begin(), cache_list.end(), [](Cache* a, Cache* b) { return a->level > b->level;});
    for (auto cp : cache_list) {
        switch (cp->type)
        {
        case CacheType::SHARED:
            computer.mem_model.add_common_interface( setup_cache( computer, *cp ) );
            break;
        case CacheType::D:
            computer.mem_model.add_data_interface( setup_cache( computer, *cp ) );
            break;
        case CacheType::I:
            computer.mem_model.add_instruction_interface( setup_cache( computer, *cp ) );
            break;
        default:
            break;
        }
    }
}
