#pragma once
#include "computer/computer.h"
#include "os_windows.h"


struct WindowsCalls {

    static void add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows );
    
    static bool load_library_exw( Computer &inter, SysCall &syscall );
    static bool get_proc_address( Computer &inter, SysCall &syscall );
    static bool get_proc_heap( Computer &inter, SysCall &syscall );
    static bool heap_alloc( Computer &inter, SysCall &syscall );
    static bool heap_free( Computer &inter, SysCall &syscall );
    /*static bool get_cmd_line_a( Computer &inter, SysCall &syscall );
    static bool get_cmd_line_w( Computer &inter, SysCall &syscall );*/
    static bool get_module_handle( Computer &inter, SysCall &syscall );
    static bool get_current_process_id( Computer &inter, SysCall &syscall );
    static bool get_sys_time_as_file_time( Computer &inter, SysCall &syscall );
    static bool query_perf_counter( Computer &inter, SysCall &syscall );
    static bool get_current_thread_id( Computer &inter, SysCall &syscall );
    static bool strlen( Computer &inter, SysCall &syscall );
    static bool strncmp( Computer &inter, SysCall &syscall );
    static bool iob_func( Computer &inter, SysCall &syscall );
    static bool abort( Computer &inter, SysCall &syscall );
    static bool fwrite( Computer &inter, SysCall &syscall );
    static bool virtual_query( Computer &inter, SysCall &syscall );
    static bool virtual_protect( Computer &inter, SysCall &syscall );
    static bool malloc( Computer &inter, SysCall &syscall );
    static bool memcpy( Computer &inter, SysCall &syscall );
};


