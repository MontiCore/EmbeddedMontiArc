#pragma once
#include "computer/computer.h"
#include "os_windows.h"

/*
    Implementation of the effect of various system functions for Windows that are required
    by the autopilots.
*/
struct WindowsCalls {
    //Registering function
    static void add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows );
    
    static bool load_library_exw( Computer &computer );
    static bool get_proc_address( Computer &computer );
    static bool get_proc_heap( Computer &computer );
    static bool heap_alloc( Computer &computer );
    static bool heap_free( Computer &computer );
    /*static bool get_cmd_line_a( Computer &computer );
    static bool get_cmd_line_w( Computer &computer );*/
    static bool get_module_handle( Computer &computer );
    static bool get_current_process_id( Computer &computer );
    static bool get_sys_time_as_file_time( Computer &computer );
    static bool query_perf_counter( Computer &computer );
    static bool get_current_thread_id( Computer &computer );
    static bool strlen( Computer &computer );
    static bool strncmp( Computer &computer );
    static bool iob_func( Computer &computer );
    static bool abort( Computer &computer );
    static bool fwrite( Computer &computer );
    static bool virtual_query( Computer &computer );
    static bool virtual_protect( Computer &computer );
    static bool malloc( Computer &computer );
    static bool memcpy( Computer &computer );
    static bool acos( Computer &computer );
};


