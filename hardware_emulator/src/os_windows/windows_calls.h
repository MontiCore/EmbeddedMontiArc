/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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


