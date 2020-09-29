#include "debug.h"
#include <iostream>
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
    Log::sys << Log::tag;
    if ( sys_call.supported() )
        Log::sys << "Supported";
    else
        Log::sys << "Unsupported";
    Log::sys << " sys_call: " << to_string( id ) << " " << sys_call.module << "!" << sys_call.name << "\n";
}

void ComputerDebug::debug_unsupp_syscall( SysCall &sys_call ) {
    if ( !unsupported_syscalls() )
        return;
    Log::sys << Log::tag << "Unsupported sys_call: " << sys_call.module << "!" << sys_call.name << "\n";
}

void ComputerDebug::debug_code( ulong addr, uint size, ulong ticks, ulong time ) {
    if ( !debug || !d_code )
        return;
        
    //Print the register updates now (for the last instruction), since there is no callback for after an instruction execution.
    if ( d_regs )
        registers->print_registers();
    else if ( d_reg_update )
        registers->print_changed_registers();
        
    Log::code << Log::tag;
    if ( decoder->succeeded ) {
        // Print current instruction pointer.
        Log::code << to_hex( addr ) << "   ";
        
        // Print binary instruction code.
        std::string res;
        for ( uint i : urange( size ) )
            res += to_hex( ( uint64_t )( decoder->code[i] ), 2 );
        char buff[128];
        sprintf( buff, "%-21s", res.c_str() );
        Log::code << buff;
        
        // Format & print the binary instruction structure to human readable format
        ZydisFormatterFormatInstruction( &formatter, &decoder->instruction, buffer.data(), buffer.size(), addr );
        sprintf( buff, "%-40s", buffer.data() );
        Log::code << buff;
    }
    Log::info << "ticks: " << ticks << " (time: " << ( time / 1000 ) << "ns)    ";
    memory->print_annotation( addr );
    Log::info << "\n";
}

void ComputerDebug::debug_code_noval( ulong addr, uint size ) {
    if ( decoder->succeeded ) {
        ZydisFormatterFormatInstruction( &formatter, &decoder->instruction, buffer.data(), buffer.size(), addr );
        Log::code << "No time for instruction: " << buffer.data() << "\n";
    }
}

void ComputerDebug::debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val ) {
    Log::err << Log::tag << "Invalid memory " << ( type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" :
             "fetch" )
             << " at " << to_hex( addr, 16, true );
             
             
    if ( type == MemAccess::WRITE )
        Log::err << " with value " << val;
    Log::err << " of size " << size << " ";
    memory->print_address_info( addr );
    Log::err << "\n";
}

void ComputerDebug::debug_mem( MemAccess type, ulong addr, uint size, slong val, ulong time ) {
    if ( !debug || !d_mem )
        return;
    //Print memory access type
    Log::LogStream *os;
    switch ( type ) {
        case MemAccess::READ: os = &Log::mem_read; break;
        case MemAccess::WRITE: os = &Log::mem_write; break;
        case MemAccess::FETCH: os = &Log::mem_fetch; break;
        default: os = &Log::info; break;
    }
    *os << Log::tag;
    
    //Print memory address
    Log::info << to_hex( addr ) << "   ";
    if ( type == MemAccess::WRITE )
        *os << to_hex( val ) << "     ";
    else {
        //Print memory content
        uint32_t s = size;
        uchar *data = ( uchar * ) memory->read_memory( addr, s );
        std::string res;
        for ( uint64_t i : ulrange( s ) ) {
            uint8_t v = *( ( uint8_t * ) & ( data[s - 1 - i] ) );
            res += to_hex( ( uint64_t )v, 2 );
        }
        char buff[128];
        sprintf( buff, "%-21s", res.c_str() );
        *os << buff;
    }
    Log::info << "                                        "
    "time: " << ( time / 1000 ) <<'.' << ( time % 1000 ) << "ns     ";
    memory->print_address_info( addr );
    Log::info << "\n";
}

void ComputerDebug::debug_call( ulong address, const char *name ) {
    if ( call() )
        Log::debug << "[CALL] " << name << "() at " << to_hex ( address ) << "\n";
}

void ComputerDebug::debug_register_syscall( SysCall const &call, ulong addr, const char *reason ) {
    if ( !debug || !d_syscalls )
        return;
        
    Log::sys << Log::tag << "Added Syscall for " << reason  << ": " << call.module << "!" << call.name << "  " << to_hex( (
                 ulong ) addr,
             3 ) << "\n";
             
    if ( undercorate_function_name( call.name, buffer )
            && call.name.compare( buffer.data() ) != 0 )
        Log::debug << buffer.data() << "\n";
}
