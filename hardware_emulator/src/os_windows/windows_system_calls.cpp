/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "os_windows/windows_system_calls.h"
#include <cmath>

using namespace std;



void WindowsSystemCalls::add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows ) {
    const char *reason = "windows";
    sys_calls.add_syscall( SysCall( "LoadLibraryExW", "KERNEL32.DLL", load_library_exw ), reason );
    sys_calls.add_syscall( SysCall( "GetProcAddress", "KERNEL32.DLL", get_proc_address ), reason );
    sys_calls.add_syscall( SysCall( "GetProcessHeap", "KERNEL32.DLL", get_proc_heap ), reason );
    sys_calls.add_syscall( SysCall( "HeapAlloc", "KERNEL32.DLL", heap_alloc ), reason );
    sys_calls.add_syscall( SysCall( "HeapFree", "KERNEL32.DLL", heap_free ), reason );
    /*sys_calls.add_syscall( SysCall( "GetCommandLineA", "KERNEL32.DLL", get_cmd_line_a ), reason );
    sys_calls.add_syscall( SysCall( "GetCommandLineW", "KERNEL32.DLL", get_cmd_line_w ), reason );*/
    sys_calls.add_syscall( SysCall( "GetModuleHandleW", "KERNEL32.DLL", get_module_handle ), reason );
    sys_calls.add_syscall( SysCall( "GetCurrentProcessId", "KERNEL32.DLL", get_current_process_id ), reason );
    sys_calls.add_syscall( SysCall( "GetSystemTimeAsFileTime", "KERNEL32.DLL", get_sys_time_as_file_time ), reason );
    sys_calls.add_syscall( SysCall( "QueryPerformanceCounter", "KERNEL32.DLL", query_perf_counter ), reason );
    sys_calls.add_syscall( SysCall( "GetCurrentThreadId", "KERNEL32.DLL", get_current_thread_id ), reason );
    sys_calls.add_syscall(SysCall("MultiByteToWideChar", "KERNEL32.DLL", MultiByteToWideChar), reason);
    
    sys_calls.add_syscall( SysCall( "strlen", "MSVCRT.DLL", strlen ), reason );
    sys_calls.add_syscall( SysCall( "strncmp", "MSVCRT.DLL", strncmp ), reason );
    sys_calls.add_syscall( SysCall( "__iob_func", "MSVCRT.DLL", iob_func ), reason );
    sys_calls.add_syscall( SysCall( "abort", "MSVCRT.DLL", abort ), reason );
    sys_calls.add_syscall( SysCall( "fwrite", "MSVCRT.DLL", fwrite ), reason );
    sys_calls.add_syscall( SysCall( "VirtualQuery", "KERNEL32.DLL", virtual_query ), reason );
    sys_calls.add_syscall( SysCall( "VirtualProtect", "KERNEL32.DLL", virtual_protect ), reason );
    sys_calls.add_syscall( SysCall( "malloc", "MSVCRT.DLL", malloc ), reason );
    sys_calls.add_syscall( SysCall( "memcpy", "MSVCRT.DLL", memcpy ), reason );
    sys_calls.add_syscall( SysCall( "acos", "MSVCRT.DLL", acos ), reason );
    sys_calls.add_syscall(SysCall("localeconv", "MSVCRT.DLL", localeconv), reason);
    sys_calls.add_syscall(SysCall("memset", "MSVCRT.DLL", memset), reason);
    sys_calls.add_syscall(SysCall("___lc_codepage_func", "MSVCRT.DLL", ___lc_codepage_func), reason);
}

bool WindowsSystemCalls::load_library_exw( Computer &computer ) {
    auto name_addr = computer.func_call_windows.get_param1_64();
    auto name_str = computer.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    if ( name.compare( "api-ms-win-core-fibers-l1-1-1" ) == 0 ||
            name.compare( "api-ms-win-core-synch-l1-2-0" ) == 0 ) {
        computer.func_call_windows.set_return_64( 0 );
        return true;
    }
    ulong handle;
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type == Symbols::Symbol::Type::HANDLE )
        handle = sym.addr;
    else {
        handle = computer.handles.add_handle( ( char * )name_str );
        computer.symbols.add_symbol( name, Symbols::Symbol::Type::HANDLE, handle );
    }
    computer.func_call_windows.set_return_64( handle );
    return true;
}
bool WindowsSystemCalls::get_proc_address( Computer &computer ) {
    auto mod = computer.func_call_windows.get_param1_64();
    auto sec_ptr = computer.memory.get_section( mod );
    if ( sec_ptr == nullptr || sec_ptr != computer.handles.section ) {
        Log::err.log_tag("Invalid Module HANDLE (%s)", to_hex(mod).c_str());
        return false;
    }
    auto note_ptr = computer.handles.section->annotations.get_annotation( mod );
    if ( note_ptr == nullptr ) {
        Log::err.log_tag("Module HANDLE does not exists (%s)", to_hex(mod).c_str());
        return false;
    }
    auto name_addr = computer.func_call_windows.get_param2_64();
    auto name_str = computer.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    
    ulong call_addr;
    auto sym = computer.symbols.get_symbol( name );
    if ( sym.type == Symbols::Symbol::Type::SYSCALL )
        call_addr = sym.addr;
    else
        call_addr = computer.sys_calls.add_syscall( SysCall( name, "", nullptr ), "get_proc_address resolve" );
    computer.func_call_windows.set_return_64( call_addr );
    return true;
}

bool WindowsSystemCalls::get_proc_heap( Computer &computer ) {
    computer.func_call_windows.set_return_64( computer.heap.heap_handle );
    return true;
}



bool WindowsSystemCalls::heap_alloc( Computer &computer ) {
    auto heap_handle = computer.func_call_windows.get_param1_64();
    if ( computer.heap.heap_handle != heap_handle ) {
        Log::err.log_tag("Trying to alloc on non-existing heap");
        return false;
    }
    auto flags = computer.func_call_windows.get_param2_64();
    auto byte_count = computer.func_call_windows.get_param3_64();
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call_windows.set_return_64( addr );
    else
        computer.func_call_windows.set_return_64( 0 );
    return true;
}

bool WindowsSystemCalls::heap_free( Computer &computer ) {
    auto heap_handle = computer.func_call_windows.get_param1_64();
    if ( computer.heap.heap_handle != heap_handle ) {
        Log::err.log_tag("Trying to alloc on non-existing heap");
        return false;
    }
    auto flags = computer.func_call_windows.get_param2_64();
    auto memory = computer.func_call_windows.get_param3_64();
    if ( computer.heap.free( memory ) )
        computer.func_call_windows.set_return_64( 1 );
    else
        computer.func_call_windows.set_return_64( 0 );
    return true;
}

//bool WindowsSystemCalls::get_cmd_line_a( Computer &computer ) {
//    computer.func_call_windows.set_return( computer.cmd_line_str );
//    return true;
//}
//
//bool WindowsSystemCalls::get_cmd_line_w( Computer &computer ) {
//    computer.func_call_windows.set_return( computer.cmd_line_wstr );
//    return true;
//}

bool WindowsSystemCalls::get_module_handle( Computer &computer ) {
    auto name_addr = computer.func_call_windows.get_param1_64();
    auto name_str = computer.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    auto sym = computer.symbols.get_symbol( name );
    ulong res = 0;
    if ( sym.type == Symbols::Symbol::Type::HANDLE )
        res = sym.addr;
    computer.func_call_windows.set_return_64( res );
    return true;
}

bool WindowsSystemCalls::get_current_process_id( Computer &computer ) {
    computer.func_call_windows.set_return_64( 0x25 );
    return true;
}

bool WindowsSystemCalls::get_sys_time_as_file_time( Computer &computer ) {
    auto addr = computer.func_call_windows.get_param1_64();
    computer.memory.write_long_word( addr, 0x123 );
    return true;
}

bool WindowsSystemCalls::query_perf_counter( Computer &computer ) {
    auto addr = computer.func_call_windows.get_param1_64();
    computer.memory.write_long_word( addr, 0x456 );
    return true;
}

bool WindowsSystemCalls::get_current_thread_id( Computer &computer ) {
    computer.func_call_windows.set_return_64( 0x29 );
    return true;
}

bool WindowsSystemCalls::strlen( Computer &computer ) {
    auto addr = computer.func_call_windows.get_param1_64();
    auto name_str = computer.memory.read_str( addr );
    string name = ( char * )name_str;
    uint size = ( uint ) name.size();
    //Log::sys << Log::tag << "strlen(\"" << name << "\"): " << to_string( size ) << "\n";
    computer.func_call_windows.set_return_64( size );
    return true;
}

bool WindowsSystemCalls::strncmp( Computer &computer ) {
    auto addr1 = computer.func_call_windows.get_param1_64();
    auto addr2 = computer.func_call_windows.get_param2_64();
    auto count = computer.func_call_windows.get_param3_64();
    string str1 = ( char * )computer.memory.read_str( addr1 );
    string str2 = ( char * )computer.memory.read_str( addr2 );
    sint res = ::strncmp( str1.c_str(), str2.c_str(), count );
    /*Log::sys << Log::tag << "strncmp(\"" << str1 << "\",\"" << str2 << "\"," + to_string( count ) << "\"): " << to_string(
                 res ) << "\n";*/
    computer.func_call_windows.set_return_64( res );
    return true;
}

bool WindowsSystemCalls::iob_func( Computer &computer ) {
    computer.func_call_windows.set_return_64( computer.io_slot.start_address );
    return true;
}

bool WindowsSystemCalls::abort( Computer &computer ) {
    computer.exit_emulation();
    return true;
}

bool WindowsSystemCalls::fwrite( Computer &computer ) {
    //size_t fwrite ( const void * ptr, size_t size, size_t count, FILE * stream );
    auto data = computer.func_call_windows.get_param1_64();
    auto size = computer.func_call_windows.get_param2_64();
    auto count = computer.func_call_windows.get_param3_64();
    auto stream = computer.func_call_windows.get_param4_64();
    string str1 = ( char * )computer.memory.read_str( data );
    //printf( "fwrite(\"%s\", %d, %d, %016" PRIX64 ")", str1.c_str(), size, count, stream );
    computer.func_call_windows.set_return_64( size * count );
    return false;
}

bool WindowsSystemCalls::virtual_query( Computer &computer ) {
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
    
    auto lpaddr = computer.func_call_windows.get_param1_64();
    auto lpbuffer = computer.func_call_windows.get_param2_64();
    auto size = computer.func_call_windows.get_param3_64();
    if ( size < sizeof( MEMORY_BASIC_INFORMATION ) )
        Log::err.log_tag("VirtualQuery with size < sizeof(MEMORY_BASIC_INFORMATION)");
    auto sec_ptr = computer.memory.get_section( lpaddr );
    if ( sec_ptr == nullptr ) {
        Log::err.log_tag("VirtualQuery outside pages");
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
    computer.func_call_windows.set_return_64( size );
    return true;
}

bool WindowsSystemCalls::virtual_protect( Computer &computer ) {
    auto lpaddr = computer.func_call_windows.get_param1_64();
    auto size = computer.func_call_windows.get_param2_64();
    auto protect = computer.func_call_windows.get_param3_64();
    //printf( "VirtualProtect(%016" PRIX64 ", %" PRIX64", %" PRIX64")", lpaddr, size, protect );
    computer.func_call_windows.set_return_64( 1 );
    return true;
}

bool WindowsSystemCalls::malloc( Computer &computer ) {
    auto byte_count = computer.func_call_windows.get_param1_64();
    //cout << "malloc(" << byte_count << ")" << endl;
    uint64_t addr;
    if ( computer.heap.alloc( byte_count, addr ) )
        computer.func_call_windows.set_return_64( addr );
    else
        computer.func_call_windows.set_return_64( 0 );
    return true;
}

bool WindowsSystemCalls::memcpy( Computer &computer ) {
    auto target_pointer = computer.func_call_windows.get_param1_64();
    auto source_pointer = computer.func_call_windows.get_param2_64();
    auto size = computer.func_call_windows.get_param3_64();
    if (computer.debug.syscalls())
        Log::sys.log_tag("memcpy(%s, %s, %llu)", to_hex(target_pointer).c_str(), to_hex(source_pointer).c_str(), size);
    auto r = computer.memory.read_memory( source_pointer, size );
    computer.memory.write_memory( target_pointer, size, r );
    return true;
}

bool WindowsSystemCalls::acos( Computer &computer ) {
    computer.func_call_windows.set_return_double( ::acos( computer.func_call_windows.get_param1_double() ) );
    return true;
}

bool WindowsSystemCalls::localeconv(Computer& computer)
{
    computer.func_call_windows.set_return_64(((OS::Windows*)computer.os.get())->lconv_slot.start_address);
    return true;
}

// void * memset ( void * ptr, int value, size_t num );
bool WindowsSystemCalls::memset(Computer& computer)
{
    auto ptr = computer.func_call_windows.get_param1_64();
    auto value = computer.func_call_windows.get_param2_32();
    auto num = computer.func_call_windows.get_param3_64();
    std::vector<uint8_t> data(num, (uint8_t)value);
    computer.memory.write_memory(ptr, num, data.data());
    computer.func_call_windows.set_return_64(ptr);
    return true;
}

//#include <Windows.h>
/*
int MultiByteToWideChar(
  UINT                              CodePage,
  DWORD                             dwFlags,
  _In_NLS_string_(cbMultiByte)LPCCH lpMultiByteStr,
  int                               cbMultiByte,
  LPWSTR                            lpWideCharStr,
  int                               cchWideChar
);

ERROR_INSUFFICIENT_BUFFER = 122L
*/
bool WindowsSystemCalls::MultiByteToWideChar(Computer& computer)
{
    auto code_page = computer.func_call_windows.get_param1_32();
    auto dwFlags = computer.func_call_windows.get_param2_32();
    auto lpMultiByteStr = computer.func_call_windows.get_param3_64();
    auto cbMultiByte = computer.func_call_windows.get_param4_32();
    //auto lpWideCharStr = computer.func_call_windows.get_param5_64();
    //auto cchWideChar = computer.func_call_windows.get_param6_32();
    if (computer.debug.syscalls())
        Log::sys.log_tag("MultiByteToWideChar(%u, %u, %s, %u)", code_page, dwFlags, to_hex(lpMultiByteStr).c_str(), cbMultiByte);
    computer.func_call_windows.set_return_64(122L); // ERROR_INSUFFICIENT_BUFFER
    return true;
}

bool WindowsSystemCalls::___lc_codepage_func(Computer& computer)
{
    computer.func_call_windows.set_return_64(1252);
    return true;
}
