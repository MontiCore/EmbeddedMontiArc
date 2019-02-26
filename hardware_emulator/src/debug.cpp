#include "debug.h"
#include <iostream>
using namespace std;

void ComputerDebug::init( Memory &mem, Registers &regs, CodeDecoder &decoder ) {
    this->mem = &mem;
    this->regs = &regs;
    this->decoder = &decoder;
    ZydisFormatterInit( &formatter, ZYDIS_FORMATTER_STYLE_INTEL );
    /*this->debug = true;
    this->d_code = true;
    this->d_regs = false;
    this->d_reg_update = true;
    this->d_mem = true;
    this->d_syscalls = true;*/
}

void ComputerDebug::debug_syscall( SysCall &sys_call, ulong id ) {
    if ( !debug || !d_syscalls )
        return;
    Log::sys << Log::tag;
    if ( sys_call.type == SysCall::SUPPORTED )
        Log::sys << "Supported";
    else
        Log::sys << "Unsupported";
    Log::sys << " sys_call: " << to_string( id ) << " " << sys_call.module << "!" << sys_call.name << "\n";
}

void ComputerDebug::debug_code( ulong addr, uint size ) {
    if ( !debug || !d_code )
        return;
        
    if ( d_regs )
        regs->print_registers();
    else if ( d_reg_update )
        regs->print_changed_registers();
        
    Log::code << Log::tag;
    if ( decoder->succeeded ) {
        // Print current instruction pointer.
        Log::code << to_hex( addr ) << "   ";
        
        std::string res;
        for ( uint i : urange( size ) )
            res += to_hex( ( uint64_t )( decoder->code[i] ), 2 );
        char buff[128];
        sprintf( buff, "%-21s", res.c_str() );
        Log::code << buff;
        
        // Format & print the binary instruction structure to human readable format
        ZydisFormatterFormatInstruction( &formatter, &decoder->instruction, buffer.begin(), buffer.size(), addr );
        sprintf( buff, "%-40s", buffer.begin() );
        Log::code << buff;
    }
    mem->print_annotation( addr );
    Log::info << "\n";
}

void ComputerDebug::debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val ) {
    Log::err << Log::tag << "Invalid memory " << ( type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" :
             "fetch" )
             << " at " << to_hex( addr, 16, true );
             
             
    if ( type == MemAccess::WRITE )
        Log::err << " with value " << val;
    Log::err << " of size " << size << " ";
    mem->print_address_info( addr );
    Log::err << "\n";
}

void ComputerDebug::debug_mem( MemAccess type, ulong addr, uint size, slong val ) {
    if ( !debug || !d_mem )
        return;
    Log::LogStream *os;
    switch ( type ) {
        case MemAccess::READ: os = &Log::mem_read; break;
        case MemAccess::WRITE: os = &Log::mem_write; break;
        case MemAccess::FETCH: os = &Log::mem_fetch; break;
        default: os = &Log::info; break;
    }
    *os << Log::tag;
    
    Log::info << to_hex( addr ) << "   ";
    if ( type == MemAccess::WRITE )
        *os << to_hex( val ) << "     ";
    else {
        uint32_t s = size;
        auto data = mem->read_memory( addr, s );
        std::string res;
        for ( uint64_t i : ulrange( s ) ) {
            uint8_t v = *( ( uint8_t * ) & ( data[s - 1 - i] ) );
            res += to_hex( ( uint64_t )v, 2 );
        }
        char buff[128];
        sprintf( buff, "%-21s", res.c_str() );
        *os << buff;
    }
    
    mem->print_address_info( addr );
    Log::info << "\n";
}

void ComputerDebug::debug_register_syscall( SysCall const &call, ulong addr ) {
    if ( !debug || !d_syscalls )
        return;
        
    Log::sys << Log::tag << "Added Syscall: " << call.module << "!" << call.name << "  " << to_hex( ( ulong ) addr,
             3 ) << "\n";
             
    if ( undercorate_function_name( call.name, buffer )
            && call.name.compare( buffer.begin() ) != 0 )
        Log::debug << buffer.begin() << "\n";
}
