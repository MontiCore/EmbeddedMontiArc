#include "os_windows/windows_calls.h"

using namespace std;



void WindowsCalls::add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows ) {
    sys_calls.add_syscall( SysCall( "LoadLibraryExW", "KERNEL32.DLL", load_library_exw ) );
    sys_calls.add_syscall( SysCall( "GetProcAddress", "KERNEL32.DLL", get_proc_address ) );
    sys_calls.add_syscall( SysCall( "GetProcessHeap", "KERNEL32.DLL", get_proc_heap ) );
    sys_calls.add_syscall( SysCall( "HeapAlloc", "KERNEL32.DLL", heap_alloc ) );
    sys_calls.add_syscall( SysCall( "HeapFree", "KERNEL32.DLL", heap_free ) );
    /*sys_calls.add_syscall( SysCall( "GetCommandLineA", "KERNEL32.DLL", get_cmd_line_a ) );
    sys_calls.add_syscall( SysCall( "GetCommandLineW", "KERNEL32.DLL", get_cmd_line_w ) );*/
    sys_calls.add_syscall( SysCall( "GetModuleHandleW", "KERNEL32.DLL", get_module_handle ) );
    sys_calls.add_syscall( SysCall( "GetCurrentProcessId", "KERNEL32.DLL", get_current_process_id ) );
    sys_calls.add_syscall( SysCall( "GetSystemTimeAsFileTime", "KERNEL32.DLL", get_sys_time_as_file_time ) );
    sys_calls.add_syscall( SysCall( "QueryPerformanceCounter", "KERNEL32.DLL", query_perf_counter ) );
    sys_calls.add_syscall( SysCall( "GetCurrentThreadId", "KERNEL32.DLL", get_current_thread_id ) );
    
    sys_calls.add_syscall( SysCall( "strlen", "MSVCRT.DLL", strlen ) );
    sys_calls.add_syscall( SysCall( "strncmp", "MSVCRT.DLL", strncmp ) );
    sys_calls.add_syscall( SysCall( "__iob_func", "MSVCRT.DLL", iob_func ) );
    sys_calls.add_syscall( SysCall( "abort", "MSVCRT.DLL", abort ) );
    sys_calls.add_syscall( SysCall( "fwrite", "MSVCRT.DLL", fwrite ) );
    sys_calls.add_syscall( SysCall( "VirtualQuery", "KERNEL32.DLL", virtual_query ) );
    sys_calls.add_syscall( SysCall( "VirtualProtect", "KERNEL32.DLL", virtual_protect ) );
    sys_calls.add_syscall( SysCall( "malloc", "MSVCRT.DLL", malloc ) );
}

bool WindowsCalls::load_library_exw( Computer &computer, SysCall &syscall ) {
    auto name_addr = computer.func_call->get_param1_64();
    auto name_str = computer.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    if ( name.compare( "api-ms-win-core-fibers-l1-1-1" ) == 0 ||
            name.compare( "api-ms-win-core-synch-l1-2-0" ) == 0 ) {
        computer.func_call->set_return_64( 0 );
        return true;
    }
    ulong handle;
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type == Symbols::Symbol::HANDLE )
        handle = sym.addr;
    else {
        handle = computer.handles.add_handle( ( char * )name_str );
        computer.symbols.add_symbol( name, Symbols::Symbol::HANDLE, handle );
    }
    computer.func_call->set_return_64( handle );
    return true;
}
bool WindowsCalls::get_proc_address( Computer &computer, SysCall &syscall ) {
    auto mod = computer.func_call->get_param1_64();
    auto sec_ptr = computer.memory.get_section( mod );
    if ( sec_ptr == nullptr || sec_ptr != computer.handles.section ) {
        Log::err << "Invalid Module HANDLE (" << to_hex( mod ) << ")\n";
        return false;
    }
    auto note_ptr = computer.handles.section->annotations.get_annotation( mod );
    if ( note_ptr == nullptr ) {
        Log::err << "Module HANDLE does not exists (" << to_hex( mod ) << ")\n";
        return false;
    }
    auto name_addr = computer.func_call->get_param2_64();
    auto name_str = computer.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    
    ulong call_addr;
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type == Symbols::Symbol::SYSCALL )
        call_addr = sym.addr;
    else
        call_addr = computer.sys_calls.add_syscall( SysCall( name, "", nullptr ) );
    computer.func_call->set_return_64( call_addr );
    return true;
}

bool WindowsCalls::get_proc_heap( Computer &computer, SysCall &syscall ) {
    computer.func_call->set_return_64( computer.heap.heap_handle );
    return true;
}



bool WindowsCalls::heap_alloc( Computer &computer, SysCall &syscall ) {
    auto heap_handle = computer.func_call->get_param1_64();
    if ( computer.heap.heap_handle != heap_handle ) {
        Log::err << "Trying to alloc on non-existing heap\n";
        return false;
    }
    auto flags = computer.func_call->get_param2_64();
    auto byte_count = computer.func_call->get_param3_64();
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call->set_return_64( addr );
    else
        computer.func_call->set_return_64( 0 );
    return true;
}

bool WindowsCalls::heap_free( Computer &computer, SysCall &syscall ) {
    auto heap_handle = computer.func_call->get_param1_64();
    if ( computer.heap.heap_handle != heap_handle ) {
        Log::err << "Trying to alloc on non-existing heap\n";
        return false;
    }
    auto flags = computer.func_call->get_param2_64();
    auto memory = computer.func_call->get_param3_64();
    if ( computer.heap.free( memory ) )
        computer.func_call->set_return_64( 1 );
    else
        computer.func_call->set_return_64( 0 );
    return true;
}

//bool WindowsCalls::get_cmd_line_a( Computer &computer, SysCall &syscall ) {
//    computer.func_call->set_return( computer.cmd_line_str );
//    return true;
//}
//
//bool WindowsCalls::get_cmd_line_w( Computer &computer, SysCall &syscall ) {
//    computer.func_call->set_return( computer.cmd_line_wstr );
//    return true;
//}

bool WindowsCalls::get_module_handle( Computer &computer, SysCall &syscall ) {
    auto name_addr = computer.func_call->get_param1_64();
    auto name_str = computer.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    auto sym = computer.symbols.get_symbol( name );
    ulong res = 0;
    if ( sym.type == sym.HANDLE )
        res = sym.addr;
    computer.func_call->set_return_64( res );
    return true;
}

bool WindowsCalls::get_current_process_id( Computer &computer, SysCall &syscall ) {
    computer.func_call->set_return_64( 0x25 );
    return true;
}

bool WindowsCalls::get_sys_time_as_file_time( Computer &computer, SysCall &syscall ) {
    auto addr = computer.func_call->get_param1_64();
    computer.memory.write_long_word( addr, 0x123 );
    return true;
}

bool WindowsCalls::query_perf_counter( Computer &computer, SysCall &syscall ) {
    auto addr = computer.func_call->get_param1_64();
    computer.memory.write_long_word( addr, 0x456 );
    return true;
}

bool WindowsCalls::get_current_thread_id( Computer &computer, SysCall &syscall ) {
    computer.func_call->set_return_64( 0x29 );
    return true;
}

bool WindowsCalls::strlen( Computer &computer, SysCall &syscall ) {
    auto addr = computer.func_call->get_param1_64();
    auto name_str = computer.memory.read_str( addr );
    string name = ( char * )name_str;
    uint size = ( uint ) name.size();
    //Log::sys << Log::tag << "strlen(\"" << name << "\"): " << to_string( size ) << "\n";
    computer.func_call->set_return_64( size );
    return true;
}

bool WindowsCalls::strncmp( Computer &computer, SysCall &syscall ) {
    auto addr1 = computer.func_call->get_param1_64();
    auto addr2 = computer.func_call->get_param2_64();
    auto count = computer.func_call->get_param3_64();
    string str1 = ( char * )computer.memory.read_str( addr1 );
    string str2 = ( char * )computer.memory.read_str( addr2 );
    sint res = ::strncmp( str1.c_str(), str2.c_str(), count );
    /*Log::sys << Log::tag << "strncmp(\"" << str1 << "\",\"" << str2 << "\"," + to_string( count ) << "\"): " << to_string(
                 res ) << "\n";*/
    computer.func_call->set_return_64( res );
    return true;
}

bool WindowsCalls::iob_func( Computer &computer, SysCall &syscall ) {
    computer.func_call->set_return_64( computer.io_slot.start_address );
    return true;
}

bool WindowsCalls::abort( Computer &computer, SysCall &syscall ) {
    computer.exit_emulation();
    return true;
}

bool WindowsCalls::fwrite( Computer &computer, SysCall &syscall ) {
    //size_t fwrite ( const void * ptr, size_t size, size_t count, FILE * stream );
    auto data = computer.func_call->get_param1_64();
    auto size = computer.func_call->get_param2_64();
    auto count = computer.func_call->get_param3_64();
    auto stream = computer.func_call->get_param4_64();
    string str1 = ( char * )computer.memory.read_str( data );
    //printf( "fwrite(\"%s\", %d, %d, %016" PRIX64 ")", str1.c_str(), size, count, stream );
    computer.func_call->set_return_64( size * count );
    return false;
}

bool WindowsCalls::virtual_query( Computer &computer, SysCall &syscall ) {
    typedef struct _MEMORY_BASIC_INFORMATION {
        ulong  BaseAddress;
        ulong  AllocationBase;
        uint  AllocationProtect;
        ulong RegionSize;
        uint  State;
        uint  Protect;
        uint  Type;
    } MEMORY_BASIC_INFORMATION, *PMEMORY_BASIC_INFORMATION;
    MEMORY_BASIC_INFORMATION mem_info;
    
    auto lpaddr = computer.func_call->get_param1_64();
    auto lpbuffer = computer.func_call->get_param2_64();
    auto size = computer.func_call->get_param3_64();
    if ( size < sizeof( MEMORY_BASIC_INFORMATION ) )
        Log::err << Log::tag << "VirtualQuery with size < sizeof(MEMORY_BASIC_INFORMATION)" << "\n";
    auto sec_ptr = computer.memory.get_section( lpaddr );
    if ( sec_ptr == nullptr ) {
        Log::err << Log::tag << "VirtualQuery outside pages" << "\n";
        return false;
    }
    //printf( "VirtualQuery(%016" PRIX64 ")", lpaddr );
    auto &sec = *sec_ptr;
    mem_info.BaseAddress = sec.address_range.start_address;
    mem_info.AllocationBase = sec.address_range.start_address;
    mem_info.AllocationProtect = 0;
    mem_info.RegionSize = 0;
    mem_info.State = 0;
    mem_info.Protect = 0;
    mem_info.Type = 0;
    computer.memory.write_memory( lpbuffer, size, ( uchar * )&mem_info );
    computer.func_call->set_return_64( size );
    return true;
}

bool WindowsCalls::virtual_protect( Computer &computer, SysCall &syscall ) {
    auto lpaddr = computer.func_call->get_param1_64();
    auto size = computer.func_call->get_param2_64();
    auto protect = computer.func_call->get_param3_64();
    //printf( "VirtualProtect(%016" PRIX64 ", %" PRIX64", %" PRIX64")", lpaddr, size, protect );
    computer.func_call->set_return_64( 1 );
    return true;
}

bool WindowsCalls::malloc( Computer &computer, SysCall &syscall ) {
    auto byte_count = computer.func_call->get_param1_64();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call->set_return_64( addr );
    else
        computer.func_call->set_return_64( 0 );
    return true;
}
