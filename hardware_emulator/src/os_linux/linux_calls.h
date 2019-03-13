#pragma once
#include "computer/computer.h"

struct LinuxCalls {

    static void add_linux_calls( SystemCalls &sys_calls );
    
    static bool malloc( Computer &computer, SysCall &syscall );
    static bool time( Computer &computer, SysCall &syscall );
    static bool srand( Computer &computer, SysCall &syscall );
    static bool posix_memalign( Computer &computer, SysCall &syscall );
    static bool sqrt( Computer &computer, SysCall &syscall );
    static bool sin( Computer &computer, SysCall &syscall );
    static bool cos( Computer &computer, SysCall &syscall );
    static bool log( Computer &computer, SysCall &syscall );
    static bool atan2( Computer &computer, SysCall &syscall );
    static bool memcpy( Computer &computer, SysCall &syscall );
};