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
    
}

bool WindowsCalls::load_library_exw( Computer &inter, SysCall &syscall ) {
    auto name_addr = inter.fast_call.arg2.get_param1();
    auto name_str = inter.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    if ( name.compare( "api-ms-win-core-fibers-l1-1-1" ) == 0 ||
            name.compare( "api-ms-win-core-synch-l1-2-0" ) == 0 ) {
        inter.fast_call.set_return( 0 );
        return true;
    }
    cout << "Module name: " << name_str << endl;
    
    auto res = inter.handles.get_handle( ( char * )name_str );
    if ( res == 0 )
        res = inter.handles.add_handle( ( char * )name_str );
    inter.fast_call.set_return( res );
    return true;
}
bool WindowsCalls::get_proc_address( Computer &inter, SysCall &syscall ) {
    auto mod = inter.fast_call.arg2.get_param1();
    auto sec_ptr = inter.memory.get_section( mod );
    if ( sec_ptr == nullptr || sec_ptr != inter.handles.section ) {
        cout << "Invalid Module HANDLE";
        printf( " (%016" PRIx64 ")\n", mod );
        return false;
    }
    auto note_ptr = inter.handles.section->annotations.get_annotation( mod );
    if ( note_ptr == nullptr ) {
        cout << "Module HANDLE does not exists";
        printf( " (%016" PRIx64 ")\n", mod );
        return false;
    }
    auto &note = *note_ptr;
    auto name_addr = inter.fast_call.arg2.get_param2();
    auto name_str = inter.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    
    /*if ( name.compare( "IiilzCiiaScinx" ) == 0 ) {
    inter.fast_call.set_return( 1 );
    return true;
    }*/
    cout << "Module ID: " << mod << " ";
    cout << " Module name: " << note.name << " ";
    cout << "Function name: " << name_str << endl;
    
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
    auto heap_handle = inter.fast_call.arg3.get_param1();
    if ( inter.heap.heap_handle != heap_handle ) {
        cerr << "Trying to alloc on non-existing heap" << endl;
        return false;
    }
    auto flags = inter.fast_call.arg3.get_param2();
    auto byte_count = inter.fast_call.arg3.get_param3();
    uint64_t addr;
    if ( inter.heap.alloc( byte_count, addr ) )
        inter.fast_call.set_return( addr );
    else
        inter.fast_call.set_return( 0 );
    return true;
}

bool WindowsCalls::heap_free( Computer &inter, SysCall &syscall ) {
    auto heap_handle = inter.fast_call.arg3.get_param1();
    if ( inter.heap.heap_handle != heap_handle ) {
        cerr << "Trying to alloc on non-existing heap" << endl;
        return false;
    }
    auto flags = inter.fast_call.arg3.get_param2();
    auto memory = inter.fast_call.arg3.get_param3();
    uint64_t addr;
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
    auto name_addr = inter.fast_call.arg1.get_param1();
    auto name_str = inter.memory.read_wstr_as_str( name_addr );
    string name = ( char * )name_str;
    auto res = inter.handles.get_handle( ( char * )name_str );
    inter.fast_call.set_return( res );
    inter.memory.print_address_info( name_addr );
    printf( "Name VA: 0x%016" PRIX64 " ", name_addr );
    cout << "Module ID: " << res << " ";
    cout << " Module name: " << name << endl;
    return true;
}