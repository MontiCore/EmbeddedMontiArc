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

#include "computer/memory.h"
#include "computer/computer.h"


struct JNIEmulator {
    Computer *computer;
    MemoryRange jni_interface_slot;
    MemoryRange jni_env_slot;
    MemoryRange buffer_slot;
    
    
    MemorySection *section;
    SectionStack *section_stack;
    
    uint current_array_size;
    ulong current_handle;
    ulong jobject_handle;
    
    JNIEmulator() : computer( nullptr ), current_array_size( 0 ), current_handle( 0 ) {}
    
    bool loaded() {
        return computer != nullptr;
    }
    
    void init( Computer &computer );
    
    bool call( ulong function_addr );
    bool call( ulong function_addr, double param );
    bool call( ulong function_addr, ulong param );
    bool call( ulong function_addr, double *buffer, uint size );
    
    double return_double();
    
    
    static bool get_array_length( Computer &inter, SysCall &syscall );
    static bool get_double_array_elements( Computer &inter, SysCall &syscall );
    static bool release_double_array_elements( Computer &inter, SysCall &syscall );
};