#pragma once
#include "computer/computer.h"

struct LinuxCalls {

    static void add_linux_calls( SystemCalls &sys_calls );
    
    static bool malloc( Computer &computer );
    static bool time( Computer &computer );
    static bool srand( Computer &computer );
    static bool posix_memalign( Computer &computer );
    static bool sqrt( Computer &computer );
    static bool sin( Computer &computer );
    static bool cos( Computer &computer );
    static bool log( Computer &computer );
    static bool atan2( Computer &computer );
    static bool memcpy( Computer &computer );
};