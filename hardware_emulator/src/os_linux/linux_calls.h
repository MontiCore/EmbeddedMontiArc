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

/*
    The system function implementations for Linux.
    If a new unregistered function calls is necessary for the functionning of the autopilot,
    it has to be implemented and registered here.
*/
struct LinuxCalls {

    //Registering function.
    static void add_linux_calls( SystemCalls &sys_calls );
    
    static bool malloc( Computer &computer );
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
};