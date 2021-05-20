/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "debug.h"
using namespace std;

void ComputerDebug::init( Memory &mem, Registers &regs, CodeDecoder &decoder ) {
    this->memory = &mem;
    this->registers = &regs;
    this->decoder = &decoder;
    ZydisFormatterInit( &formatter, ZYDIS_FORMATTER_STYLE_INTEL );
}

void ComputerDebug::debug_syscall( SysCall &sys_call, ulong id ) {
    if ( !debug || !d_syscalls )
        return;

    Log::sys.log_tag("%s sys_call: %llu %s!%s", sys_call.supported() ? "Supported" : "Unsupported", id, sys_call.module.c_str(), sys_call.name.c_str());
}

void ComputerDebug::debug_unsupp_syscall( SysCall &sys_call ) {
    if ( !unsupported_syscalls() )
        return;
    Log::sys.log_tag("Unsupported sys_call: %s!%s", sys_call.module.c_str(), sys_call.name.c_str());
}

void ComputerDebug::debug_code( ulong addr, uint size, ulong ticks, ulong time ) {
    if ( !debug || !d_code )
        return;
        
    //Print the register updates now (for the last instruction), since there is no callback for after an instruction execution.
    if ( d_regs )
        registers->print_registers();
    else if ( d_reg_update )
        registers->print_changed_registers();
        
    char annot[256];
    memory->print_annotation(addr, annot, 256);
    if ( decoder->succeeded ) {
        // Print binary instruction code.
        std::string res;
        for ( uint i : urange( size ) )
            res += to_hex( ( uint64_t )( decoder->code[i] ), 2 );
        // Format & print the binary instruction structure to human readable format
        ZydisFormatterFormatInstruction( &formatter, &decoder->instruction, buffer.data(), buffer.size(), addr );
        Log::code.log_tag("%s   %-21s%-40sticks: %llu (time: %lluns)    %s", to_hex(addr).c_str(), res.c_str(), buffer.data(), ticks, (time / 1000), annot);
    }
    else {
        Log::code.log_tag("ticks: %llu (time: %lluns)    %s", ticks, (time / 1000), annot);
    }
}

void ComputerDebug::debug_code_noval( ulong addr, uint size ) {
    if ( decoder->succeeded ) {
        ZydisFormatterFormatInstruction( &formatter, &decoder->instruction, buffer.data(), buffer.size(), addr );
        Log::code.log("No time for instruction: %s", buffer.data());
    }
}

void ComputerDebug::debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val ) {
    char addr_info[256];
    memory->print_address_info(addr, addr_info, 256);
    if (type == MemAccess::WRITE) {
        Log::err.log_tag("Invalid memory %s at %s with value %ill of size %ul %s", 
            (type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" : "fetch"),
            to_hex(addr, 16, true).c_str(),
            val,
            size,
            addr_info
        );
    }
    else {
        Log::err.log_tag("Invalid memory %s at %s of size %ul %s",
            (type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" : "fetch"),
            to_hex(addr, 16, true).c_str(),
            size,
            addr_info
        );
    }
}

void ComputerDebug::debug_mem( MemAccess type, ulong addr, uint size, slong val, ulong time ) {
    if ( !debug || !d_mem )
        return;
    //Print memory access type
    const char* tag;
    switch ( type ) {
        case MemAccess::READ: tag = Log::mem_read.tag; break;
        case MemAccess::WRITE: tag = Log::mem_write.tag; break;
        case MemAccess::FETCH: tag = Log::mem_fetch.tag; break;
        default: tag = Log::info.tag; break;
    }
    
    
    char addr_info[256];
    memory->print_address_info(addr, addr_info, 256);
    if (type == MemAccess::WRITE) {
        Log::info.log("%-7s%s   %s                                             time: %llu.%lluns     %s",
            tag, to_hex(addr).c_str(), to_hex(val).c_str(), time / 1000, time % 1000);
    }
    else {
        //Print memory content
        uint32_t s = size;
        uchar* data = (uchar*)memory->read_memory(addr, s);
        std::string res;
        for (uint64_t i : ulrange(s)) {
            uint8_t v = *((uint8_t*) & (data[s - 1 - i]));
            res += to_hex((uint64_t)v, 2);
        }
        Log::info.log("%-7s%s   %-21s                                        time: %llu.%lluns     %s",
            tag, to_hex(addr).c_str(), res.c_str(), time / 1000, time % 1000, addr_info);
    }

}

void ComputerDebug::debug_call( ulong address, const char *name ) {
    if ( call() )
        Log::debug.log("[CALL] %s() at %s", name, to_hex(address).c_str());
}

void ComputerDebug::debug_register_syscall( SysCall const &call, ulong addr, const char *reason ) {
    if ( !debug || !d_syscalls )
        return;
        
    Log::sys.log_tag("Added Syscall for %s: %s!%s  %s", reason, call.module.c_str(), call.name.c_str(), to_hex((ulong)addr, 3).c_str());
             
    if ( undercorate_function_name( call.name, buffer )
            && call.name.compare( buffer.data() ) != 0 )
        Log::debug.log("%s", buffer.data());
}
