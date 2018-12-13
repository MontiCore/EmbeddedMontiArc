#pragma once
#include "computer/computer.h"
#include "os_windows.h"


struct WindowsCalls {

    void add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows );
    
    static bool load_library_exw( Computer &inter, SysCall &syscall );
    static bool get_proc_address( Computer &inter, SysCall &syscall );
    static bool get_proc_heap( Computer &inter, SysCall &syscall );
    static bool heap_alloc( Computer &inter, SysCall &syscall );
    static bool heap_free( Computer &inter, SysCall &syscall );
    /*static bool get_cmd_line_a( Computer &inter, SysCall &syscall );
    static bool get_cmd_line_w( Computer &inter, SysCall &syscall );*/
    static bool get_module_handle( Computer &inter, SysCall &syscall );
};


