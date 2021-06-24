/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "computer/computer.h"
#include "os_windows.h"

/*
    Implementation of the effect of various system functions for Windows that are required
    by the autopilots.
*/
struct WindowsSystemCalls {
    //Registering function
    static void add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows );
};


