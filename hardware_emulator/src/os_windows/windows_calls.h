#pragma once
#include "computer/computer.h"
#include "os_windows.h"


struct WindowsCalls {

    static void add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows );
    
    static bool load_library_exw( Computer &inter );
    static bool get_proc_address( Computer &inter );
    static bool get_proc_heap( Computer &inter );
    static bool heap_alloc( Computer &inter );
    static bool heap_free( Computer &inter );
    /*static bool get_cmd_line_a( Computer &inter );
    static bool get_cmd_line_w( Computer &inter );*/
    static bool get_module_handle( Computer &inter );
    static bool get_current_process_id( Computer &inter );
    static bool get_sys_time_as_file_time( Computer &inter );
    static bool query_perf_counter( Computer &inter );
    static bool get_current_thread_id( Computer &inter );
    static bool strlen( Computer &inter );
    static bool strncmp( Computer &inter );
    static bool iob_func( Computer &inter );
    static bool abort( Computer &inter );
    static bool fwrite( Computer &inter );
    static bool virtual_query( Computer &inter );
    static bool virtual_protect( Computer &inter );
    static bool malloc( Computer &inter );
    static bool memcpy( Computer &inter );
};


