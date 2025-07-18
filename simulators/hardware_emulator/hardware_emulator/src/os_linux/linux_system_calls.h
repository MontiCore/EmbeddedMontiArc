/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "computer/computer.h"

/*
    The system function implementations for Linux.
    If a new unregistered function calls is necessary for the functionning of the autopilot,
    it has to be implemented and registered here.
*/
struct LinuxSystemCalls {

    //Registering function.
    static void add_linux_calls( SystemCalls &sys_calls );
    
    static bool malloc( Computer &computer );
    static bool operatornew( Computer &computer );
    static bool time( Computer &computer );
    static bool srand( Computer &computer );
    static bool posix_memalign( Computer &computer );
    static bool sqrt( Computer &computer );
    static bool sin( Computer &computer );
    static bool cos( Computer &computer );
    static bool acos( Computer &computer );
    static bool exp( Computer &computer );
    static bool log( Computer &computer );
    static bool atan2( Computer &computer );
    static bool memcpy( Computer &computer );
    static bool memset( Computer &computer );
    static bool strtoll( Computer &computer );
    static bool strtod( Computer &computer );
};
