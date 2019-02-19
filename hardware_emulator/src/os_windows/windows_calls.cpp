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

bool WindowsCalls::load_library_exw( Computer &inter, SysCall &syscall ) {
    auto name_addr = inter.fast_call.get_param1();
    auto name_str = inter.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    if ( name.compare( "api-ms-win-core-fibers-l1-1-1" ) == 0 ||
            name.compare( "api-ms-win-core-synch-l1-2-0" ) == 0 ) {
        inter.fast_call.set_return( 0 );
        return true;
    }
    
    auto res = inter.handles.get_handle( ( char * )name_str );
    if ( res == 0 )
        res = inter.handles.add_handle( ( char * )name_str );
    inter.fast_call.set_return( res );
    return true;
}
bool WindowsCalls::get_proc_address( Computer &inter, SysCall &syscall ) {
    auto mod = inter.fast_call.get_param1();
    auto sec_ptr = inter.memory.get_section( mod );
    if ( sec_ptr == nullptr || sec_ptr != inter.handles.section ) {
        Log::err << "Invalid Module HANDLE (" << to_hex( mod ) << ")\n";
        return false;
    }
    auto note_ptr = inter.handles.section->annotations.get_annotation( mod );
    if ( note_ptr == nullptr ) {
        Log::err << "Module HANDLE does not exists (" << to_hex( mod ) << ")\n";
        return false;
    }
    auto &note = *note_ptr;
    auto name_addr = inter.fast_call.get_param2();
    auto name_str = inter.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    
    auto existing_call = inter.sys_calls.get_syscall( note.name, name );
    if ( existing_call == 0 ) {
        SysCall call( name, note.name, nullptr );
        existing_call = inter.sys_calls.add_syscall( SysCall( name, note.name, nullptr ) );
    }
    inter.fast_call.set_return( existing_call );
    return true;
}

bool WindowsCalls::get_proc_heap( Computer &inter, SysCall &syscall ) {
    inter.fast_call.set_return( inter.heap.heap_handle );
    return true;
}



bool WindowsCalls::heap_alloc( Computer &inter, SysCall &syscall ) {
    auto heap_handle = inter.fast_call.get_param1();
    if ( inter.heap.heap_handle != heap_handle ) {
        Log::err << "Trying to alloc on non-existing heap\n";
        return false;
    }
    auto flags = inter.fast_call.get_param2();
    auto byte_count = inter.fast_call.get_param3();
    uint64_t addr;
    if ( inter.heap.alloc( byte_count, addr ) )
        inter.fast_call.set_return( addr );
    else
        inter.fast_call.set_return( 0 );
    return true;
}

bool WindowsCalls::heap_free( Computer &inter, SysCall &syscall ) {
    auto heap_handle = inter.fast_call.get_param1();
    if ( inter.heap.heap_handle != heap_handle ) {
        Log::err << "Trying to alloc on non-existing heap\n";
        return false;
    }
    auto flags = inter.fast_call.get_param2();
    auto memory = inter.fast_call.get_param3();
    if ( inter.heap.free( memory ) )
        inter.fast_call.set_return( 1 );
    else
        inter.fast_call.set_return( 0 );
    return true;
}

//bool WindowsCalls::get_cmd_line_a( Computer &inter, SysCall &syscall ) {
//    inter.fast_call.set_return( inter.cmd_line_str );
//    return true;
//}
//
//bool WindowsCalls::get_cmd_line_w( Computer &inter, SysCall &syscall ) {
//    inter.fast_call.set_return( inter.cmd_line_wstr );
//    return true;
//}

bool WindowsCalls::get_module_handle( Computer &inter, SysCall &syscall ) {
    auto name_addr = inter.fast_call.get_param1();
    auto name_str = inter.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    auto res = inter.handles.get_handle( ( char * )name_str );
    inter.fast_call.set_return( res );
    return true;
}

bool WindowsCalls::get_current_process_id( Computer &inter, SysCall &syscall ) {
    inter.fast_call.set_return( 0x25 );
    return true;
}

bool WindowsCalls::get_sys_time_as_file_time( Computer &inter, SysCall &syscall ) {
    auto addr = inter.fast_call.get_param1();
    inter.memory.write_long_word( addr, 0x123 );
    return true;
}

bool WindowsCalls::query_perf_counter( Computer &inter, SysCall &syscall ) {
    auto addr = inter.fast_call.get_param1();
    inter.memory.write_long_word( addr, 0x456 );
    return true;
}

bool WindowsCalls::get_current_thread_id( Computer &inter, SysCall &syscall ) {
    inter.fast_call.set_return( 0x29 );
    return true;
}

bool WindowsCalls::strlen( Computer &inter, SysCall &syscall ) {
    auto addr = inter.fast_call.get_param1();
    auto name_str = inter.memory.read_str( addr );
    string name = ( char * )name_str;
    uint size = ( uint ) name.size();
    //Log::sys << Log::tag << "strlen(\"" << name << "\"): " << to_string( size ) << "\n";
    inter.fast_call.set_return( size );
    return true;
}

bool WindowsCalls::strncmp( Computer &inter, SysCall &syscall ) {
    auto addr1 = inter.fast_call.get_param1();
    auto addr2 = inter.fast_call.get_param2();
    auto count = inter.fast_call.get_param3();
    string str1 = ( char * )inter.memory.read_str( addr1 );
    string str2 = ( char * )inter.memory.read_str( addr2 );
    sint res = ::strncmp( str1.c_str(), str2.c_str(), count );
    /*Log::sys << Log::tag << "strncmp(\"" << str1 << "\",\"" << str2 << "\"," + to_string( count ) << "\"): " << to_string(
                 res ) << "\n";*/
    inter.fast_call.set_return( res );
    return true;
}

bool WindowsCalls::iob_func( Computer &inter, SysCall &syscall ) {
    inter.fast_call.set_return( OS::io_slot.start_address );
    return true;
}

bool WindowsCalls::abort( Computer &inter, SysCall &syscall ) {
    inter.exit_emulation();
    return true;
}

bool WindowsCalls::fwrite( Computer &inter, SysCall &syscall ) {
    //size_t fwrite ( const void * ptr, size_t size, size_t count, FILE * stream );
    auto data = inter.fast_call.get_param1();
    auto size = inter.fast_call.get_param2();
    auto count = inter.fast_call.get_param3();
    auto stream = inter.fast_call.get_param4();
    string str1 = ( char * )inter.memory.read_str( data );
    //printf( "fwrite(\"%s\", %d, %d, %016" PRIX64 ")", str1.c_str(), size, count, stream );
    inter.fast_call.set_return( size * count );
    return false;
}

bool WindowsCalls::virtual_query( Computer &inter, SysCall &syscall ) {
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
    
    auto lpaddr = inter.fast_call.get_param1();
    auto lpbuffer = inter.fast_call.get_param2();
    auto size = inter.fast_call.get_param3();
    if ( size < sizeof( MEMORY_BASIC_INFORMATION ) )
        Log::err << Log::tag << "VirtualQuery with size < sizeof(MEMORY_BASIC_INFORMATION)" << "\n";
    auto sec_ptr = inter.memory.get_section( lpaddr );
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
    inter.memory.write_memory( lpbuffer, size, ( uchar * )&mem_info );
    inter.fast_call.set_return( size );
    return true;
}

bool WindowsCalls::virtual_protect( Computer &inter, SysCall &syscall ) {
    auto lpaddr = inter.fast_call.get_param1();
    auto size = inter.fast_call.get_param2();
    auto protect = inter.fast_call.get_param3();
    //printf( "VirtualProtect(%016" PRIX64 ", %" PRIX64", %" PRIX64")", lpaddr, size, protect );
    inter.fast_call.set_return( 1 );
    return true;
}

bool WindowsCalls::malloc( Computer &inter, SysCall &syscall ) {
    auto byte_count = inter.fast_call.get_param1();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( inter.heap.alloc( byte_count, addr ) )
        inter.fast_call.set_return( addr );
    else
        inter.fast_call.set_return( 0 );
    return true;
}
