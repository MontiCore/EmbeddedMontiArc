#pragma once
#include "computer/computer.h"

struct LinuxCalls {

    static void add_linux_calls( SystemCalls &sys_calls );
    
    static bool malloc( Computer &inter, SysCall &syscall );
};