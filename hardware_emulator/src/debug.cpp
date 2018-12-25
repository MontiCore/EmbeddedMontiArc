#include "debug.h"
using namespace std;

void ComputerDebug::init( Memory &mem, Registers &regs, ZydisDecoder &decoder ) {
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
    Utility::color_sys();
    cout << "[S] ";
    if ( sys_call.type == SysCall::SUPPORTED ) {
        cerr << "Supported sys_call: " << id;
        cerr << " " << sys_call.module << "!" << sys_call.name;
        cerr << endl;
    }
    else {
        cerr << "Unsupported sys_call: " << id;
        cerr << " " << sys_call.module << "!" << sys_call.name;
        cerr << endl;
    }
}

void ComputerDebug::debug_code( ulong addr, uint size ) {
    if ( !debug || !d_code )
        return;
        
    if ( d_regs )
        regs->print_registers();
    else if ( d_reg_update )
        regs->print_changed_registers();
        
    Utility::color_code();
    cout << "[I] ";
    
    ZyanUSize offset = 0;
    const ZyanUSize length = size;
    ZydisDecodedInstruction instruction;
    auto code = mem->read_memory( addr, size );
    if ( ZYAN_SUCCESS( ZydisDecoderDecodeBuffer( decoder, code, length, &instruction ) ) ) {
        // Print current instruction pointer.
        Utility::color_def();
        printf( "%016" PRIX64 "   ", addr );
        
        for ( uint i : Range( size ) )
            printf( "%02" PRIX64, ( uint64_t )code[i] );
        sint printed = size * 2;
        auto to_print = 16 + 5 - printed;
        if ( to_print < 0 )
            to_print = 0;
        for ( uint i : Range( to_print ) )
            printf( " " );
            
        // Format & print the binary instruction structure to human readable format
        Utility::color_code();
        ZydisFormatterFormatInstruction( &formatter, &instruction, buffer.begin(), buffer.size(), addr );
        //puts( buffer.begin() );
        printf( "%-40s", buffer.begin() );
    }
    mem->print_annotation( addr );
    printf( "\n" );
}

void ComputerDebug::debug_mem_err( MemAccess type, MemAccessError err, ulong addr, uint size, slong val ) {

    Utility::color_err();
    mem->print_address_info( addr );
    Utility::color_err();
    cout << "\n>>> Invalid memory " << ( type == MemAccess::READ ? "read" : type == MemAccess::WRITE ? "write" : "fetch" )
         << " at 0x";
    printf( "%016" PRIX64 "  ", addr );
    
    
    if ( type == MemAccess::WRITE )
        cout << " with value " << val;
    cout << " of size " << size << endl;
}

void ComputerDebug::debug_mem( MemAccess type, ulong addr, uint size, slong val ) {
    if ( !debug || !d_mem )
        return;
        
    switch ( type ) {
        case MemAccess::READ: Utility::color_mem_read(); break;
        case MemAccess::WRITE: Utility::color_mem_write(); break;
        case MemAccess::FETCH: Utility::color_mem_fetch(); break;
        default: Utility::color_err(); break;
    }
    switch ( type ) {
        case MemAccess::READ: cout << "[R] "; break;
        case MemAccess::WRITE: cout << "[W] "; break;
        case MemAccess::FETCH: cout << "[F] "; break;
        default: cout << "[M] "; break;
    }
    
    printf( "%016" PRIX64 "   ", addr );
    switch ( type ) {
        case MemAccess::READ: Utility::color_mem_read(); break;
        case MemAccess::WRITE: Utility::color_mem_write(); break;
        case MemAccess::FETCH: Utility::color_mem_fetch(); break;
        default: Utility::color_err(); break;
    }
    if ( type == MemAccess::WRITE ) {
        printf( "%016" PRIX64 "", val );
        cout << "     ";
    }
    else {
        uint32_t s = size;
        auto data = mem->read_memory( addr, s );
        //cout << setfill( '0' ) << setw( 2 ) << hex;
        for ( uint64_t i : Range( s ) ) {
            uint8_t v = *( ( uint8_t * ) & ( data[s - 1 - i] ) );
            //cout << ( uint64_t )v << " ";
            printf( "%02" PRIX64 "", ( uint64_t )v );
        }
        sint printed = size * 2;
        auto to_print = 16 + 5 - printed;
        if ( to_print < 0 )
            to_print = 0;
        for ( uint i : Range( to_print ) )
            printf( " " );
    }
    
    mem->print_address_info( addr );
    cout << endl;
}

void ComputerDebug::debug_register_syscall( SysCall const &call, ulong addr ) {
    if ( !debug || !d_syscalls )
        return;
    Utility::color_mem_write();
    cout << "Added Syscall: " << call.module << "!" << call.name;
    printf( "  %03" PRIx32 "\n", ( uint )addr );
    
    if ( undercorate_function_name( call.name, buffer )
            && call.name.compare( buffer.begin() ) != 0 ) {
        // UnDecorateSymbolName returned success
        Utility::color_reg();
        printf( "%s\n", buffer.begin() );
    }
}
